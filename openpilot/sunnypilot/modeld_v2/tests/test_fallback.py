import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2 import modeld as modeld_module
from openpilot.sunnypilot.models import manager as manager_module


class FakeParams:
  def __init__(self):
    self.values = {}
    self.blocking_bool_writes = []

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)
    if block:
      self.blocking_bool_writes.append((key, bool(value)))

  def put(self, key, value, block=False):
    self.values[key] = value

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def test_initial_big_model_failure_falls_back_to_small():
  params = FakeParams()
  small_model = object()

  def load_big():
    raise RuntimeError("USB AMD unavailable")

  model, fallback = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=load_big,
    load_small=lambda: small_model,
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is small_model
  assert fallback is small_model
  assert params.values["ChestnutActive"] is False
  assert params.values["ChestnutLoading"] is False


def test_missing_small_pkl_does_not_kill_successful_big_model(monkeypatch):
  params = FakeParams()
  big_model = object()
  monkeypatch.setattr(modeld_module, "load_with_timeout", lambda load, timeout: load())

  model, fallback = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=lambda: big_model,
    load_small=lambda: (_ for _ in ()).throw(AssertionError("No driving pkl found")),
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is big_model
  assert fallback is None
  assert params.values["ChestnutActive"] is True


def test_successful_big_model_keeps_preloaded_small_for_runtime_fallback(monkeypatch):

  params = FakeParams()
  big_model = object()
  small_model = object()
  calls = {"big": 0, "small": 0}
  monkeypatch.setattr(modeld_module, "load_with_timeout", lambda load, timeout: load())

  def load_big():
    calls["big"] += 1
    return big_model

  def load_small():
    calls["small"] += 1
    return small_model

  model, fallback = modeld_module.load_models_with_fallback(
    chestnut=True,
    load_big=load_big,
    load_small=load_small,
    params=params,
    update_loading_progress=lambda _progress: None,
  )

  assert model is big_model
  assert fallback is small_model
  assert calls == {"big": 1, "small": 1}
  assert params.values["ChestnutActive"] is True
  assert params.values["ChestnutLoading"] is False


def test_runtime_big_model_failure_switches_to_preloaded_small():
  params = FakeParams()
  params.values["ChestnutActive"] = True
  small_model = object()
  chestnut_state = type("ChestnutState", (), {"big": True})()

  class FailingBigModel:
    def run(self, *_args, **_kwargs):
      raise RuntimeError("non-finite model output")

  active, output, fell_back = modeld_module.run_model_with_fallback(
    FailingBigModel(), small_model, params, chestnut_state, (), {}, {}, False,
  )

  assert active is small_model
  assert output is None
  assert fell_back
  assert params.values["ChestnutActive"] is False
  assert chestnut_state.big is False
  assert ("ChestnutActive", False) in params.blocking_bool_writes


def test_non_finite_big_model_plan_becomes_fallback_error():
  outputs = {"plan": np.array([np.nan])}

  with pytest.raises(RuntimeError, match="not finite"):
    modeld_module.validate_model_outputs(chestnut=True, outputs=outputs)


def test_missing_qcom_selection_queues_exact_default_fallback_ref():
  params = FakeParams()
  params.values["ModelManager_ActiveBundleChestnut"] = {
    "internalName": "BMV4",
    "minimumSelectorVersion": 18,
  }

  manager_module.ensure_default_qcom_fallback(params)

  assert params.values["ModelManager_DownloadRef"] == "5b6436a90cf6902b8aaa71c2b6f3d7164d8ae391"


@pytest.mark.parametrize("existing", (
  {"ModelManager_ActiveBundle": {"internalName": "USER", "minimumSelectorVersion": 18}},
  {"ModelManager_DownloadRef": "user-request"},
))
def test_default_fallback_never_overwrites_user_model_or_download(existing):
  params = FakeParams()
  params.values.update(existing)
  params.values["ModelManager_ActiveBundleChestnut"] = {
    "internalName": "BMV4",
    "minimumSelectorVersion": 18,
  }

  manager_module.ensure_default_qcom_fallback(params)

  if "ModelManager_DownloadRef" in existing:
    assert params.values["ModelManager_DownloadRef"] == "user-request"
  else:
    assert "ModelManager_DownloadRef" not in params.values
