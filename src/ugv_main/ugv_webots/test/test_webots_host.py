# Copyright 2026 S-H-E-R-P-AI-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the Webots host lookup used under WSL and the macOS VM."""

import pytest

from ugv_webots import webots_host


@pytest.fixture
def env(monkeypatch):
    for name in ('UGV_WEBOTS_HOST', 'WEBOTS_SHARED_FOLDER'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(webots_host, 'default_gateway', lambda: '172.20.0.1')
    return monkeypatch


def test_override_wins(env):
    env.setenv('UGV_WEBOTS_HOST', '10.1.2.3')
    env.setattr(webots_host, 'is_wsl', lambda: True)
    assert webots_host.webots_host() == '10.1.2.3'


def test_native_linux_runs_webots_locally(env):
    env.setattr(webots_host, 'is_wsl', lambda: False)
    assert webots_host.webots_host() is None


def test_wsl_mirrored_uses_loopback(env):
    env.setattr(webots_host, 'is_wsl', lambda: True)
    env.setattr(webots_host, 'wsl_networking_mode', lambda: 'mirrored')
    assert webots_host.webots_host() == '127.0.0.1'


def test_wsl_nat_uses_gateway(env):
    env.setattr(webots_host, 'is_wsl', lambda: True)
    env.setattr(webots_host, 'wsl_networking_mode', lambda: 'nat')
    assert webots_host.webots_host() == '172.20.0.1'


def test_wsl_without_gateway_falls_back_to_loopback(env):
    env.setattr(webots_host, 'is_wsl', lambda: True)
    env.setattr(webots_host, 'wsl_networking_mode', lambda: None)
    env.setattr(webots_host, 'default_gateway', lambda: None)
    assert webots_host.webots_host() == '127.0.0.1'


def test_macos_shared_folder_uses_gateway(env):
    env.setenv('WEBOTS_SHARED_FOLDER', '/Users/me/shared:/home/me/shared')
    assert webots_host.webots_host() == '172.20.0.1'
