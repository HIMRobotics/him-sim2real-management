"""Viewer dependency integrity and offline reuse."""

import hashlib
import io
from unittest.mock import patch

import pytest

from containers.sim import viewer_assets


def test_cache_download_repair_and_offline_reuse(tmp_path):
    data = b"test module"
    assets = {"build/module.js": hashlib.sha256(data).hexdigest()}
    with patch.multiple(viewer_assets, CACHE=tmp_path, ASSETS=assets):
        with patch.object(viewer_assets, "urlopen", return_value=io.BytesIO(data)) as fetch:
            paths = viewer_assets.install()
            fetch.assert_called_once()
        assert paths[0].read_bytes() == data
        with patch.object(viewer_assets, "urlopen", side_effect=OSError("offline")) as fetch:
            assert viewer_assets.install() == paths
            fetch.assert_not_called()
        paths[0].write_bytes(b"corrupt")
        with patch.object(viewer_assets, "urlopen", return_value=io.BytesIO(data)):
            assert viewer_assets.install()[0].read_bytes() == data


@pytest.mark.parametrize("response", [b"wrong version", OSError("offline")])
def test_failed_download_is_not_cached(tmp_path, response):
    assets = {"build/module.js": hashlib.sha256(b"expected").hexdigest()}
    options = (
        {"side_effect": response} if isinstance(response, OSError)
        else {"return_value": io.BytesIO(response)}
    )
    with (
        patch.multiple(viewer_assets, CACHE=tmp_path, ASSETS=assets),
        patch.object(viewer_assets, "urlopen", **options),
        pytest.raises(RuntimeError, match="before viewing offline"),
    ):
        viewer_assets.install()
    assert not list(tmp_path.iterdir())
