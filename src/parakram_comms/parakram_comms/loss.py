"""
Seeded application-level packet loss (CLAUDE_CODE/05, the primary loss mechanism).

A ``LossFilter`` sits on a robot's subscriptions to PEER coordination traffic (``/<peer>/state``,
``/<peer>/intent``) and decides, before the message is processed, whether to drop it:

    if not loss_filter.accept(peer, 'intent'):
        return                           # dropped: the coordination layer never sees it

Every (receiver, sender, topic) stream has its own RNG, seeded from the run's integer seed and
the stream name, so the drop decisions are a deterministic function of the seed and the order of
the messages on that stream (a re-run with the same seed drops the same message indices).

Two models, both with mean loss ``loss``:

* ``bernoulli`` (default): every message is dropped independently with probability ``loss``.
* ``gilbert_elliott``: bursty (CLAUDE_CODE/07, per arXiv 2609.13711): a two-state Markov chain,
  GOOD (deliver) / BAD (drop), with stationary P(BAD) = ``loss`` and lag-1 correlation of the
  drop indicator ``burst_corr`` (0.8 by default): P(G->B) = loss (1 - rho),
  P(B->G) = (1 - loss) (1 - rho); mean burst length 1 / P(B->G).

The counters also keep the first / last ``seq`` seen on a stream (state and intent carry one),
so a log tells three numbers apart: what the sender published (the seq span), what the transport
delivered (``received``; a RELIABLE, retransmitting transport would keep it at the seq span
whatever the network does) and what coordination processed (``passed``). ``drop_bursts``
counts runs of consecutive drops, so ``dropped / drop_bursts`` is the realized mean burst length
(Bernoulli: 1 / (1 - loss); Gilbert-Elliott: 1 / P(B->G)).

The loss is applied to state/intent only (BEST_EFFORT in ``qos``), never to the RELIABLE task /
award traffic of CLAUDE_CODE/04, and never to anything the reactive safety layer reads (it reads
no peer traffic at all). Counters per stream are appended to ``comms_<robot>.csv`` so every run
records the loss it actually applied.
"""

import csv
import hashlib
import os
import random

MODELS = ('bernoulli', 'gilbert_elliott')
LOG_COLUMNS = ['t', 'receiver', 'sender', 'topic', 'received', 'dropped', 'passed',
               'first_seq', 'last_seq', 'drop_bursts', 'loss', 'model', 'burst_corr', 'seed']


def stream_seed(seed, *names):
    """Stable 64-bit seed for one stream (independent of Python's per-process hash salt)."""
    text = '|'.join([str(int(seed))] + [str(n) for n in names])
    return int.from_bytes(hashlib.sha256(text.encode()).digest()[:8], 'big')


class DropProcess:
    """Drop decisions of one stream."""

    def __init__(self, loss, rng, model='bernoulli', burst_corr=0.8):
        """Create the process; ``rng`` is the stream's ``random.Random``."""
        self.loss, self.rng, self.model = float(loss), rng, model
        if model == 'gilbert_elliott':
            self.p_gb = self.loss * (1.0 - burst_corr)
            self.p_bg = (1.0 - self.loss) * (1.0 - burst_corr)
            self.bad = rng.random() < self.loss          # start in the stationary distribution

    def drop(self):
        """Return True if the next message of the stream is lost."""
        if self.loss <= 0.0:
            return False
        if self.model == 'bernoulli':
            return self.rng.random() < self.loss
        u = self.rng.random()
        self.bad = (u >= self.p_bg) if self.bad else (u < self.p_gb)
        return self.bad


class LossFilter:
    """A receiver's loss filter over its peer streams."""

    def __init__(self, loss, seed, receiver, model='bernoulli', burst_corr=0.8, log_path=None):
        """Validate the setting; ``log_path``: the CSV the counters are appended to."""
        loss = float(loss)
        if not 0.0 <= loss < 1.0:
            raise ValueError(f'loss must be in [0, 1), got {loss}')
        if model not in MODELS:
            raise ValueError(f'loss model must be one of {MODELS}, got {model!r}')
        if not 0.0 <= burst_corr < 1.0:
            raise ValueError(f'burst_corr must be in [0, 1), got {burst_corr}')
        self.loss, self.seed, self.receiver = loss, int(seed), receiver
        self.model, self.burst_corr = model, float(burst_corr)
        self.streams, self.counts = {}, {}
        self.log_path = log_path
        if log_path and not os.path.exists(log_path):
            with open(log_path, 'w', newline='') as f:
                csv.writer(f).writerow(LOG_COLUMNS)

    def accept(self, sender, topic, seq=None):
        """Count one received message; return False if it must be dropped (not processed)."""
        key = (sender, topic)
        proc = self.streams.get(key)
        if proc is None:
            rng = random.Random(stream_seed(self.seed, self.receiver, sender, topic))
            proc = self.streams[key] = DropProcess(self.loss, rng, self.model, self.burst_corr)
            # received, dropped, first seq, last seq, drop bursts, last message dropped
            self.counts[key] = [0, 0, None, None, 0, False]
        count = self.counts[key]
        count[0] += 1
        if seq is not None:
            if count[2] is None:
                count[2] = seq
            count[3] = seq
        if proc.drop():
            count[1] += 1
            if not count[5]:
                count[4] += 1
            count[5] = True
            return False
        count[5] = False
        return True

    def wrap(self, sender, topic, callback):
        """Return a subscription callback that drops before ``callback`` processes the message."""
        def filtered(msg):
            if self.accept(sender, topic, getattr(msg, 'seq', None)):
                callback(msg)
        return filtered

    def rows(self, t):
        """Return the current cumulative counters, one row per stream."""
        return [[f'{t:.3f}', self.receiver, sender, topic, rec, drop, rec - drop,
                 '' if first is None else first, '' if last is None else last, bursts,
                 self.loss, self.model, self.burst_corr, self.seed]
                for (sender, topic), (rec, drop, first, last, bursts, _)
                in sorted(self.counts.items())]

    def log(self, t):
        """Append the current counters to the CSV (no-op without a log path)."""
        if self.log_path and self.counts:
            with open(self.log_path, 'a', newline='') as f:
                csv.writer(f).writerows(self.rows(t))
