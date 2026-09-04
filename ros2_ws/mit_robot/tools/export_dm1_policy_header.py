#!/usr/bin/env python3
"""Export frozen DM1 DreamWaQ actor to a dependency-free C++ header.

The checkpoint is a regular PyTorch training checkpoint rather than a
TorchScript module.  The running MuJoCo binary deliberately does not depend
on Python or libtorch, so this script is executed by CMake and emits only the
weights needed by ``ActorCritic_DWAQ.act_inference``.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import torch


EXPECTED = {
    "actor.0.weight": (512, 64),
    "actor.0.bias": (512,),
    "actor.2.weight": (256, 512),
    "actor.2.bias": (256,),
    "actor.4.weight": (128, 256),
    "actor.4.bias": (128,),
    "actor.6.weight": (12, 128),
    "actor.6.bias": (12,),
    "vae.encoder.encoder.0.weight": (128, 270),
    "vae.encoder.encoder.0.bias": (128,),
    "vae.encoder.encoder.2.weight": (64, 128),
    "vae.encoder.encoder.2.bias": (64,),
    "vae.latent_mu.weight": (16, 64),
    "vae.latent_mu.bias": (16,),
    "vae.vel_mu.weight": (3, 64),
    "vae.vel_mu.bias": (3,),
}
# 换模型要修改的地方：新 checkpoint 的 SHA256
EXPECTED_SHA256 = "5a05f95dd5e7ddef83a48a076415267ef22bee6dad55fcb43b92a5f2adf4ab08"


def format_array(values: list[float], indent: str = "  ") -> str:
    formatted = [f"{value:.9g}F" for value in values]
    lines = []
    for start in range(0, len(formatted), 8):
        lines.append(indent + ", ".join(formatted[start:start + 8]))
    return ",\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checkpoint_sha256 = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
    if checkpoint_sha256 != EXPECTED_SHA256:
        raise ValueError(
            f"unexpected model_3960 SHA256: {checkpoint_sha256} != {EXPECTED_SHA256}"
        )

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    state = checkpoint.get("model_state_dict")
    if not isinstance(state, dict):
        raise ValueError("checkpoint does not contain model_state_dict")

    tensors: dict[str, list[float]] = {}
    for name, shape in EXPECTED.items():
        if name not in state:
            raise ValueError(f"missing required tensor: {name}")
        tensor = state[name].detach().cpu().contiguous()
        if tuple(tensor.shape) != shape:
            raise ValueError(
                f"unexpected shape for {name}: {tuple(tensor.shape)} != {shape}"
            )
        if tensor.dtype != torch.float32:
            raise ValueError(f"unexpected dtype for {name}: {tensor.dtype}")
        tensors[name] = tensor.reshape(-1).tolist()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        # 换模型要修改的地方：模型编号和生成头文件 namespace
        output.write("// Generated from model_3960.pt; do not edit.\n")
        output.write("#pragma once\n\n#include <array>\n\n")
        output.write("namespace dm1_policy_3960 {\n\n")
        output.write("inline constexpr bool kSquashActionMean = false;\n\n")
        for name, shape in EXPECTED.items():
            identifier = "k_" + name.replace(".", "_")
            values = tensors[name]
            output.write(
                f"inline constexpr std::array<float, {len(values)}> {identifier}{{{{\n"
            )
            output.write(format_array(values))
            output.write("\n}};\n\n")
        output.write("}  // namespace dm1_policy_3960\n")


if __name__ == "__main__":
    main()
