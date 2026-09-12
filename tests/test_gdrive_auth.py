# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Offline tests for the Google Drive installed-app auth helpers.

The one-shot OAuth flow cannot run without a browser and a real Google account,
so only the pure steps are pinned: reading the downloaded client secret, building
the consent URL, and writing the token file the upload path reads. Nothing here
opens a socket, a browser, or a real credential file.
"""

from __future__ import annotations

import json
import urllib.parse
from pathlib import Path

import pytest

from scripts.dev import gdrive_auth


def _client_secret(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "client_secret.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _token_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "nomad" / "gdrive_token.json"
    monkeypatch.setattr(gdrive_auth, "TOKEN_PATH", path)
    return path


def test_load_client_credentials_reads_installed_app_secret(tmp_path: Path):
    secret = _client_secret(tmp_path, {"installed": {"client_id": "id-1", "client_secret": "secret-1"}})

    assert gdrive_auth.load_client_credentials(secret) == ("id-1", "secret-1")


def test_load_client_credentials_reads_flat_secret(tmp_path: Path):
    secret = _client_secret(tmp_path, {"client_id": "id-2", "client_secret": "secret-2"})

    assert gdrive_auth.load_client_credentials(secret) == ("id-2", "secret-2")


def test_load_client_credentials_rejects_a_secret_without_credentials(tmp_path: Path):
    secret = _client_secret(tmp_path, {"installed": {"client_id": "id-3"}})

    with pytest.raises(KeyError):
        gdrive_auth.load_client_credentials(secret)


def test_build_auth_url_requests_offline_consent():
    query = urllib.parse.parse_qs(urllib.parse.urlparse(gdrive_auth.build_auth_url("id-4")).query)

    assert query["client_id"] == ["id-4"]
    assert query["response_type"] == ["code"]
    assert query["access_type"] == ["offline"]
    assert query["prompt"] == ["consent"]
    assert query["scope"] == [gdrive_auth.SCOPE]
    assert query["redirect_uri"] == [gdrive_auth.REDIRECT_URI]


def test_auth_url_carries_only_the_public_client_identifier():
    # The consent URL reaches the browser and the operator's history, so it must
    # carry no client_secret and no other parameter beyond the consent request.
    query = urllib.parse.parse_qs(urllib.parse.urlparse(gdrive_auth.build_auth_url("id-5")).query)

    assert set(query) == {"client_id", "redirect_uri", "response_type", "scope", "access_type", "prompt"}
    assert gdrive_auth.build_auth_url("id-5") == gdrive_auth.build_auth_url("id-5")


def test_write_token_file_records_defaults_and_folder_ids(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    path = _token_file(tmp_path, monkeypatch)
    gdrive_auth.TASK1_FOLDER_ID = "task1-folder"
    monkeypatch.setattr(gdrive_auth, "TASK2_FOLDER_ID", "task2-folder")

    written = gdrive_auth.write_token_file(
        {"access_token": "access-1", "refresh_token": "refresh-1"}, "id-6", "secret-6"
    )

    assert written == path
    token = json.loads(path.read_text(encoding="utf-8"))
    assert token == {
        "access_token": "access-1",
        "refresh_token": "refresh-1",
        "client_id": "id-6",
        "client_secret": "secret-6",
        "token_type": "Bearer",
        "expires_in": 3600,
        "scope": gdrive_auth.SCOPE,
        "folder_id": "task1-folder",
        "task2_folder_id": "task2-folder",
    }


def test_write_token_file_keeps_values_the_token_endpoint_returned(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    path = _token_file(tmp_path, monkeypatch)

    gdrive_auth.write_token_file(
        {"access_token": "access-2", "refresh_token": "refresh-2", "token_type": "MAC", "expires_in": 60},
        "id-7",
        "secret-7",
    )

    token = json.loads(path.read_text(encoding="utf-8"))
    assert token["token_type"] == "MAC"
    assert token["expires_in"] == 60
