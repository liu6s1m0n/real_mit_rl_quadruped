#!/usr/bin/env python3
"""Generate fixed Python reference vectors for the frozen model_4210 actor."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


CASES = (
    ("stand", 0.0, 0.0, 0.0),
    ("forward", 0.25, 0.0, 0.0),
    ("left", 0.0, 0.25, 0.0),
    ("right", 0.0, -0.25, 0.0),
)


def linear(value: torch.Tensor, state: dict[str, torch.Tensor], name: str) -> torch.Tensor:
    return torch.nn.functional.linear(
        value, state[f"{name}.weight"], state[f"{name}.bias"]
    )


def make_observation(vx: float, vy: float, wz: float) -> list[float]:
    value = [0.0] * 45
    value[0:3] = [2.0 * vx, 2.0 * vy, 0.25 * wz]
    value[3:6] = [0.01, -0.02, 0.03]
    value[6:9] = [0.0, 0.0, -1.0]
    value[9:21] = [0.004 * (index - 5.5) for index in range(12)]
    value[21:33] = [0.002 * (index - 5.5) for index in range(12)]
    value[33:45] = [0.001 * ((index % 5) - 2) for index in range(12)]
    return value


def format_values(values: list[float]) -> str:
    def format_value(value: float) -> str:
        text = f"{value:.9g}"
        if "." not in text and "e" not in text and "E" not in text:
            text += ".0"
        return text + "F"

    return ", ".join(format_value(value) for value in values)


def format_matrix(matrix: list[list[float]]) -> str:
    rows = ["  {{" + format_values(row) + "}}" for row in matrix]
    return ",\n".join(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    state = checkpoint["model_state_dict"]
    observations: list[list[float]] = []
    histories: list[list[float]] = []
    expected_actions: list[list[float]] = []

    with torch.no_grad():
        for _, vx, vy, wz in CASES:
            observation = make_observation(vx, vy, wz)
            history = []
            for frame in range(6):
                frame_value = list(observation)
                offset = 0.0005 * (frame - 5)
                for index in range(9, 45):
                    frame_value[index] += offset * (1.0 + (index % 4))
                history.extend(frame_value)
            history_tensor = torch.tensor(history, dtype=torch.float32).reshape(1, 270)
            observation_tensor = torch.tensor(observation, dtype=torch.float32).reshape(1, 45)
            encoded = torch.nn.functional.elu(
                linear(history_tensor, state, "vae.encoder.encoder.0")
            )
            encoded = linear(encoded, state, "vae.encoder.encoder.2")
            latent = linear(encoded, state, "vae.latent_mu")
            velocity = linear(encoded, state, "vae.vel_mu")
            actor_input = torch.cat((latent, velocity, observation_tensor), dim=1)
            actor = torch.nn.functional.elu(linear(actor_input, state, "actor.0"))
            actor = torch.nn.functional.elu(linear(actor, state, "actor.2"))
            actor = torch.nn.functional.elu(linear(actor, state, "actor.4"))
            action = linear(actor, state, "actor.6").reshape(-1).tolist()
            observations.append(observation)
            histories.append(history)
            expected_actions.append(action)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        output.write("// Generated from Python model_4210 reference inference.\n")
        output.write("#pragma once\n\n#include <array>\n\n")
        output.write("namespace dm1_policy_4210_golden {\n\n")
        output.write("inline constexpr std::array<std::array<float, 45>, 4> kObservations{{\n")
        output.write(format_matrix(observations))
        output.write("\n}};\n\n")
        output.write("inline constexpr std::array<std::array<float, 270>, 4> kHistories{{\n")
        output.write(format_matrix(histories))
        output.write("\n}};\n\n")
        output.write("inline constexpr std::array<std::array<float, 12>, 4> kExpectedActions{{\n")
        output.write(format_matrix(expected_actions))
        output.write("\n}};\n\n}")
        output.write("  // namespace dm1_policy_4210_golden\n")


if __name__ == "__main__":
    main()
