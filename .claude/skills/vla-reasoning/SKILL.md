# Vision-Language-Action Reasoning Skill

## Skill Name
vla-reasoning

## Purpose
Provide expert instruction on Vision-Language-Action (VLA) systems for robotics, including Whisper speech recognition, LLM-based task planning (GPT-4, Qwen), vision-language grounding (CLIP, LLaVA), action grounding to ROS 2, multi-modal perception fusion, and safety validation.

## When to Use
- Student asks about integrating LLMs with robotic control
- Student needs help with Whisper ASR integration to ROS 2
- Student requests prompt engineering guidance for robotics tasks
- Student encounters issues with CLIP or LLaVA vision-language models
- Student asks about action grounding (converting LLM plans to ROS 2 actions)
- Student needs multi-modal fusion strategies (vision + language + proprioception)
- Student asks about safety validation for LLM-generated commands

## Inputs
- Student question about VLA architecture or implementation
- Whisper transcription issues or latency problems
- LLM prompts requiring optimization or debugging
- CLIP/LLaVA object grounding challenges
- Action grounding code needing review
- Safety concerns about LLM-generated commands

## Outputs
- Whisper ROS 2 node implementation examples
- LLM prompt templates for robotic task decomposition
- CLIP object detection integration code
- LLaVA visual question answering examples
- Action grounding modules (JSON parsing → ROS 2 action goals)
- Safety validator implementations (workspace limits, collision checking)
- Multi-modal fusion architectures
- References to Module 4 documentation

## Constraints / Boundaries
- Support both cloud LLMs (GPT-4 via OpenAI API) and local models (Qwen via Transformers)
- Provide cost estimates for API usage (~$0.05 per command, $5 for course)
- Use standard ROS 2 action types where possible (nav2_msgs/NavigateToPose)
- Include safety validation (prevent invalid or dangerous commands)
- Handle perception failures gracefully (retry, request clarification, timeout)
- Target >60% end-to-end success rate for complete VLA pipelines

## Linked Documentation
- `docs/module4-vla/vla-systems.md` - Complete VLA guide (Whisper, LLM, CLIP, LLaVA, action grounding, multi-modal fusion, safety)
- `specs/001-embodied-ai-book/contracts/ros2-message-contracts.md` - Custom message definitions (TaskPlan, TaskStep, TaskStatus)
- Constitution: Safety, reliability, graceful degradation standards

## Example Skill Invocation
**Student Query**: "My LLM generates plans with invalid actions. How do I constrain outputs?"

**Skill Response**:
1. **System Prompt Constraints**: Explicitly list allowed actions in prompt:
   ```
   Actions ONLY: navigate_to, detect_object, grasp_object, place_object, report_status
   If user requests other actions, output: {"error": "Action not supported"}
   ```
2. **Output Format Enforcement**: Require JSON structure, provide schema:
   ```json
   {"steps": [{"action": "navigate_to", "params": {...}}]}
   ```
3. **Post-Processing Validation**: Parse JSON, check each action against whitelist
4. **Safety Validator Node**: Implement separate validator (see code example in Module 4)
5. **Few-Shot Examples**: Include 3-5 example command→plan pairs in prompt
6. **Lower Temperature**: Use `temperature=0.2` for more deterministic outputs
7. Reference: See `docs/module4-vla/vla-systems.md` section "Prompt Engineering for Robotics"
