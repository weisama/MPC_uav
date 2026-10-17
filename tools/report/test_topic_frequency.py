"""`topic_frequency.summarize` 的合成用例（不读包、不依赖 ROS）。"""

import numpy as np
import pytest

from topic_frequency import summarize


def test_regular_50hz():
    t = 100.0 + np.arange(0.0, 2.0, 0.02)          # 100 帧，理想 50 Hz
    s = summarize(t)
    assert s["n"] == 100
    assert s["span_s"] == pytest.approx(1.98)
    assert s["rate_n_over_span_hz"] == pytest.approx(100 / 1.98)
    assert s["median_rate_hz"] == pytest.approx(50.0)
    assert s["max_gap_ms"] == pytest.approx(20.0)


def test_gap_is_visible_in_max_but_not_median():
    t = np.array([0.0, 0.02, 0.04, 0.20])           # 中间断 160 ms
    s = summarize(t)
    assert s["median_dt_ms"] == pytest.approx(20.0)
    assert s["max_gap_ms"] == pytest.approx(160.0)
    assert s["rate_n_over_span_hz"] == pytest.approx(4 / 0.20)


def test_bursty_stream_n_over_span_and_median_differ():
    # 成簇：三帧几乎同时到，再等 1 s；中位间隔很小，n/span 才是出包率。
    t = np.array([0.0, 1e-4, 2e-4, 1.0, 1.0001, 1.0002])
    s = summarize(t)
    assert s["rate_n_over_span_hz"] == pytest.approx(6 / 1.0002)
    assert s["median_rate_hz"] > 1000.0


def test_degenerate_inputs():
    for t in (np.array([]), np.array([5.0])):
        s = summarize(t)
        assert s["n"] == (0 if t.size == 0 else 1)
        assert np.isnan(s["median_rate_hz"])
        assert np.isnan(s["max_gap_ms"])
