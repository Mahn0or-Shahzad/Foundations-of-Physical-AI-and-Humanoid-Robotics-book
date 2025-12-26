"""
LLM Answer Generation
Uses OpenAI Chat Completion API to generate answers based on retrieved context
"""

import os
from typing import Dict, Optional
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()


class LLMGenerator:
    """Manages OpenAI chat completions for answer generation"""

    def __init__(self):
        """Initialize OpenAI client"""
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key or self.openai_api_key == 'your_free_openai_api_key':
            raise ValueError("OPENAI_API_KEY not configured in .env file")

        self.client = OpenAI(api_key=self.openai_api_key)

        # Use gpt-4o-mini for cost-effective, high-quality responses
        # Free tier: $5 credit, ~$0.0015 per 1K input tokens, ~$0.006 per 1K output tokens
        self.model = os.getenv('OPENAI_CHAT_MODEL', 'gpt-4o-mini')

        print(f"✓ LLMGenerator initialized (model: {self.model})")

    def generate_answer(
        self,
        context_text: str,
        user_question: str,
        selected_text: Optional[str] = None
    ) -> Dict:
        """
        Generate answer to user question using retrieved context.

        Args:
            context_text: Formatted context from RAG retrieval (from Qdrant)
            user_question: The question asked by the user
            selected_text: Optional highlighted text from the page

        Returns:
            Dictionary containing:
            {
                'answer': Generated answer text,
                'model': Model used,
                'tokens_used': Token count (for cost tracking),
                'sources': List of source references
            }
        """
        print(f"\nGenerating answer for: '{user_question}'")
        print(f"Context length: {len(context_text)} characters")

        try:
            # Build system prompt
            system_prompt = self._build_system_prompt()

            # Build user prompt with context
            user_prompt = self._build_user_prompt(
                context_text=context_text,
                user_question=user_question,
                selected_text=selected_text
            )

            # TODO: Uncomment to make actual OpenAI API call
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,  # Balanced creativity vs consistency
                max_tokens=1000,  # Limit response length
            )
            
            # Extract answer
            answer = response.choices[0].message.content
            tokens_used = response.usage.total_tokens

            # Placeholder response (no API call)
#             print("  ⚠ Using placeholder answer (API call not yet activated)")
#             answer = f"""[Placeholder Answer]

# Question: {user_question}

# Based on the retrieved context, here's what the book says:
# {context_text[:300]}...

# (This is a placeholder. Actual OpenAI API call is commented out to avoid charges.)
# """
#             tokens_used = 0

            # Extract source references from context
            sources = self._extract_sources(context_text)

            result = {
                'answer': answer,
                'model': self.model,
                'tokens_used': tokens_used,
                'sources': sources
            }

            print(f"  ✓ Answer generated ({tokens_used} tokens)")
            return result

        except Exception as e:
            print(f"  ✗ Error generating answer: {e}")
            raise

    def _build_system_prompt(self) -> str:
        """
        Build system prompt for the chatbot.
        Defines the bot's behavior, tone, and constraints.
        """
        return """You are an expert teaching assistant for the book "AI in Motion: Foundations of Physical AI and Humanoid Robotics."

Your role:
- Answer questions about ROS 2, digital twin simulation, Isaac ROS, and vision-language-action systems
- Use ONLY information from the provided book context
- Be precise, technical, and educational
- Cite specific sections when possible
- If information is not in the context, say so clearly

Tone:
- Academic but accessible
- Encourage hands-on learning
- Reference specific modules and chapters
- Provide code examples when relevant

Constraints:
- Do NOT invent information not in the book
- Do NOT provide general robotics advice outside the book's scope
- If asked about topics not covered, direct to relevant external resources
- Keep answers concise (2-4 paragraphs unless detail requested)
"""

    def _build_user_prompt(
        self,
        context_text: str,
        user_question: str,
        selected_text: Optional[str] = None
    ) -> str:
        """
        Build user prompt combining context and question.

        Args:
            context_text: Retrieved context from Qdrant
            user_question: User's question
            selected_text: Optional highlighted text

        Returns:
            Formatted prompt for LLM
        """
        if selected_text:
            # User highlighted text - provide extra context
            prompt = f"""The user is reading the book and highlighted the following text:

\"\"\"
{selected_text}
\"\"\"

They asked: "{user_question}"

Here is relevant context from the book:

{context_text}

Please answer their question based on the highlighted text and the provided context from the book. Be specific and reference the relevant sections.
"""
        else:
            # General question - use retrieved context
            prompt = f"""The user asked: "{user_question}"

Here is relevant context from the book:

{context_text}

Please answer their question based on the provided context from the book. Cite specific sections or modules when helpful.
"""

        return prompt

    def _extract_sources(self, context_text: str) -> list:
        """
        Extract source references from formatted context text.

        Args:
            context_text: Formatted context with [Source N: ...] markers

        Returns:
            List of source strings
        """
        import re

        # Extract [Source N: file.md - section] patterns
        source_pattern = r'\[Source \d+: (.+?)\]'
        sources = re.findall(source_pattern, context_text)

        return sources if sources else ["General book content"]


if __name__ == "__main__":
    # Test the LLM generator
    print("=" * 60)
    print("Testing LLM Generator")
    print("=" * 60)
    print()

    try:
        generator = LLMGenerator()

        # Sample context (simulated retrieval result)
        sample_context = """[Source 1: module1-ros2/ros2-architecture.md - What is ROS 2?]
ROS 2 is an open-source middleware framework providing the communication infrastructure,
tools, and libraries necessary to build complex robotic systems.

------------------------------------------------------------

[Source 2: module1-ros2/ros2-architecture.md - Topics]
Topics implement the publish-subscribe pattern for continuous, many-to-many data streams.
"""

        # Test answer generation
        result = generator.generate_answer(
            context_text=sample_context,
            user_question="What is ROS 2 and how does it use topics?",
            selected_text=None
        )

        print("\nGenerated response:")
        print(f"  Answer: {result['answer'][:200]}...")
        print(f"  Model: {result['model']}")
        print(f"  Tokens: {result['tokens_used']}")
        print(f"  Sources: {result['sources']}")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("\nMake sure OPENAI_API_KEY is configured in .env!")
