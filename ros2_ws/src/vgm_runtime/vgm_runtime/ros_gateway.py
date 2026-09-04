"""ROS gateway that publishes only validator-minted high-level skills."""

from __future__ import annotations

import json

from .coordinator import TaskCoordinator
from .openai_intent import OpenAIIntentModel, load_local_api_key
from .pipeline import IntentPipeline
from .serialization import plan_to_mapping, scene_from_mapping
from .validator import SkillValidator


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SkillGateway(Node):
        def __init__(self) -> None:
            super().__init__("skill_gateway")
            load_local_api_key()
            validator = SkillValidator()
            self.pipeline = IntentPipeline(
                OpenAIIntentModel(schema=validator.schema),
                validator,
                TaskCoordinator(validator.policy),
            )
            self.scene = None
            self.validated_publisher = self.create_publisher(
                String, "/vgm/validated_skill", 10
            )
            self.status_publisher = self.create_publisher(String, "/vgm/status", 10)
            self.create_subscription(String, "/vgm/grounded_scene", self.on_scene, 10)
            self.create_subscription(String, "/vgm/transcript", self.on_transcript, 10)

        def on_scene(self, message: String) -> None:
            try:
                self.scene = scene_from_mapping(json.loads(message.data))
            except Exception as error:
                self.publish_status(False, "invalid_scene", type(error).__name__)

        def on_transcript(self, message: String) -> None:
            if self.scene is None:
                self.publish_status(False, "scene_required", "no grounded scene")
                return
            decision = self.pipeline.decide(message.data, self.scene)
            self.publish_status(decision.accepted, decision.code, decision.message)
            if not decision.accepted or decision.validated_skill is None:
                return
            output = {
                "validated_skill": decision.validated_skill.to_mapping(),
                "plan": plan_to_mapping(decision.plan),
            }
            self.validated_publisher.publish(
                String(data=json.dumps(output, sort_keys=True))
            )

        def publish_status(self, accepted: bool, code: str, message: str) -> None:
            self.status_publisher.publish(
                String(
                    data=json.dumps(
                        {"accepted": accepted, "code": code, "message": message},
                        sort_keys=True,
                    )
                )
            )

    rclpy.init()
    node = SkillGateway()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
