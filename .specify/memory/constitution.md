# Embodied AI & Humanoid Robotics Book Constitution

<!--
Sync Impact Report:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Version Change: INITIAL → 1.0.0
Date: 2025-12-19

Changes Summary:
- Initial constitution created for research-grade technical book project
- Established 8 core principles for scientific rigor and reproducibility
- Defined technical standards, authoring workflow, and governance rules
- All placeholder tokens filled with concrete values

Principles Established:
1. Scientific Correctness & Rigor
2. Reproducibility & Verifiability
3. Academic Citation Standards
4. Conceptual Clarity & Accessibility
5. Specification-First Authoring
6. Code Quality & Executability
7. Plagiarism Zero-Tolerance
8. Version Control & Traceability

Templates Status:
✅ plan-template.md - Constitution Check section compatible
✅ spec-template.md - User scenarios and requirements align with book chapters
✅ tasks-template.md - Task categorization supports book development workflow

Follow-up Actions:
- None required - all placeholders filled
- Constitution ready for immediate use
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
-->

## Core Principles

### I. Scientific Correctness & Rigor

Every technical and factual assertion in this book MUST be grounded in validated scientific and engineering knowledge. This is NON-NEGOTIABLE.

**Requirements:**
- All claims about robotics algorithms, control theory, kinematics, dynamics, and AI methods MUST be verifiable against authoritative sources
- Mathematical formulations (Forward/Inverse Kinematics, Rigid Body Dynamics, Zero Moment Point, Model Predictive Control, etc.) MUST be numerically validated where feasible
- Robotics conventions (DH parameters, coordinate frames, notation) MUST follow established standards (IEEE, ACM, Springer Robotics)
- Cross-validation required against peer-reviewed literature, standard textbooks, and established frameworks (ROS 2, MuJoCo, PyBullet, etc.)
- Any novel interpretations or synthesis MUST be clearly distinguished from established knowledge

**Rationale:** This book serves as an educational and reference resource for students, practitioners, and researchers. Scientific accuracy is the foundation of trust and utility.

### II. Reproducibility & Verifiability

All implementations, simulations, and experimental results described in the book MUST be reproducible by readers.

**Requirements:**
- Every code snippet MUST be executable and tested (Python, ROS 2, simulation frameworks)
- Algorithms MUST include sufficient detail for independent implementation
- Mathematical derivations MUST show key steps and reference equations
- Simulation setups MUST specify: environment parameters, robot models, controller gains, initial conditions
- Where applicable, provide links to reference implementations, datasets, or simulation files
- Equations MUST be accompanied by clear variable definitions and units

**Rationale:** Reproducibility is a core tenet of scientific practice and enables readers to build upon the knowledge presented.

### III. Academic Citation Standards

All sources MUST be properly cited following APA (7th edition) guidelines.

**Requirements:**
- Minimum 50% of references from peer-reviewed sources (IEEE Transactions, ACM conferences, Science Robotics, IJRR, etc.)
- Minimum total of 25 academically validated and properly cited sources across the manuscript
- Every technical claim requiring support MUST include inline citations
- Reference list MUST be complete with: authors, year, title, venue, DOI/URL where applicable
- Citations MUST be traceable—readers must be able to locate and verify sources
- Gray literature (technical reports, preprints) acceptable when appropriately contextualized

**Rationale:** Proper attribution upholds academic integrity and enables readers to explore topics in depth.

### IV. Conceptual Clarity & Accessibility

Technical content MUST balance precision with pedagogical clarity.

**Requirements:**
- Target readability: Flesch-Kincaid Grade Level 10-12 (precise yet accessible technical prose)
- Assume reader background: intermediate engineering/CS (undergraduate or early graduate level)
- Complex concepts MUST be introduced progressively with clear definitions
- Jargon MUST be defined on first use; maintain a glossary if needed
- Visual aids (diagrams, kinematic chains, control block diagrams, state machines) MUST clarify complex systems
- Each chapter MUST include: learning objectives, key equations/algorithms, summary, and references

**Rationale:** The book must be understandable to its target audience while maintaining technical depth.

### V. Specification-First Authoring

Book development follows the Spec-Driven Development (SDD) methodology using Spec-Kit Plus.

**Requirements:**
- Every chapter or major section MUST begin with a specification (`specs/<feature>/spec.md`) defining scope, learning objectives, and content requirements
- Architectural decisions (framework choices, chapter organization, tooling) MUST be documented in planning artifacts (`specs/<feature>/plan.md`)
- Content development MUST proceed from specifications to implementation (writing), following defined tasks (`specs/<feature>/tasks.md`)
- Changes to scope or structure MUST be reflected in updated specification files
- Prompt History Records (PHRs) MUST be created for significant authoring sessions

**Rationale:** Specification-first ensures clarity of intent, traceability, and systematic content development.

### VI. Code Quality & Executability

All code presented in the book MUST be correct, tested, and executable.

**Requirements:**
- Code snippets MUST be syntactically correct and runnable in specified environments
- Dependencies MUST be clearly stated (Python version, ROS 2 version, simulation frameworks)
- Code MUST follow language-specific best practices (PEP 8 for Python, ROS 2 conventions)
- Non-trivial algorithms MUST include comments explaining key steps
- Where applicable, provide complete example scripts or Jupyter notebooks
- Code MUST NOT introduce security vulnerabilities or unsafe practices
- Code examples MUST align with principles of simplicity and clarity—no unnecessary complexity

**Rationale:** Executable code examples are essential for learning and practical application.

### VII. Plagiarism Zero-Tolerance

Original authorship and proper attribution are absolute requirements.

**Requirements:**
- Plagiarism detection MUST be run on all content before final publication
- Zero tolerance for unattributed copying of text, code, equations, or diagrams
- When adapting or summarizing existing work, proper paraphrasing and citation MUST be used
- Direct quotations MUST be clearly marked and cited
- Code adapted from external sources MUST include attribution and license compatibility check
- Any similarities flagged by plagiarism detection MUST be resolved before merge or release

**Rationale:** Academic and ethical integrity are non-negotiable.

### VIII. Version Control & Traceability

All manuscript evolution MUST be tracked and documented.

**Requirements:**
- All content MUST be version-controlled using Git
- Semantic versioning MUST be applied to the manuscript: MAJOR.MINOR.PATCH
  - MAJOR: Significant restructuring, new chapters, major content overhauls
  - MINOR: New sections, expanded coverage, added examples
  - PATCH: Corrections, clarifications, minor edits
- Changes MUST be committed with clear, descriptive commit messages
- Chapter-level and section-level changes MUST reference issue numbers or specification files
- Architectural decisions (chapter structure, tooling, framework choices) MUST be documented as ADRs when significant

**Rationale:** Traceability ensures accountability, facilitates collaboration, and maintains content integrity.

## Technical & Writing Standards

### Reference and Citation Policy

- **APA 7th Edition**: Follow APA (7th edition) for all citations and references
- **Peer-reviewed minimum**: At least 50% of references must come from peer-reviewed venues (IEEE, ACM, Springer, Nature, Science Robotics, IJRR, ICRA, IROS, RSS)
- **Total reference minimum**: At least 25 academically validated and properly cited sources
- **Inline citation requirement**: Every technical or factual assertion requiring support must include inline citations
- **Traceability**: All citations must be verifiable—readers must be able to locate sources
- **Gray literature**: Technical reports, preprints, and documentation acceptable when contextualized appropriately

### Mathematical and Algorithmic Standards

- **Notation consistency**: Use standard robotics notation (DH parameters, homogeneous transformations, Jacobians, etc.)
- **Derivation completeness**: Show key steps in mathematical derivations; provide references for omitted details
- **Validation**: Core equations (FK, IK, dynamics, control laws) must be numerically validated where feasible
- **Variable definitions**: All variables, symbols, and units must be clearly defined
- **Algorithm pseudocode**: Non-trivial algorithms must include clear pseudocode or structured descriptions

### Code Standards

- **Language and versions**: Specify Python version (e.g., Python 3.10+), ROS 2 version (e.g., Humble), simulation frameworks (MuJoCo, PyBullet, Unity)
- **Executability**: All code must be executable and tested
- **Style guides**: Follow PEP 8 for Python, ROS 2 conventions for robotics code
- **Comments**: Non-trivial code must include explanatory comments
- **Dependencies**: Clearly list all dependencies with version constraints
- **Security**: No insecure practices (hardcoded secrets, SQL injection, XSS, etc.)

### Readability and Prose Quality

- **Target audience**: Intermediate to advanced engineering/computer science students and practitioners
- **Flesch-Kincaid Grade Level**: 10-12 (precise technical prose accessible to target audience)
- **Jargon handling**: Define jargon on first use; maintain glossary if needed
- **Progressive complexity**: Introduce concepts in logical order with clear prerequisites
- **Visual aids**: Use diagrams, kinematic chains, control architectures, and state machines to clarify complex content

### Manuscript Constraints

- **Length**: 10,000-15,000 words total
- **Minimum references**: 25 academically validated sources
- **Plagiarism**: Zero tolerance—content must pass plagiarism detection with zero overlap
- **Expert review**: Manuscript must withstand expert-level review in AI, control systems, and humanoid robotics before v1.0

## Authoring Workflow & Tooling

### Spec-Driven Development Workflow

1. **Specification**: Define chapter or section scope in `specs/<chapter>/spec.md`
   - Learning objectives
   - Content requirements (concepts, equations, algorithms, examples)
   - Success criteria (what readers should be able to do after reading)

2. **Planning**: Document architectural decisions in `specs/<chapter>/plan.md`
   - Chapter structure
   - Key topics and their sequencing
   - Mathematical formulations to cover
   - Code examples and simulations required
   - Cross-references to other chapters

3. **Task Breakdown**: Create actionable tasks in `specs/<chapter>/tasks.md`
   - Writing tasks (sections, subsections)
   - Equation derivation and validation tasks
   - Code example development and testing tasks
   - Diagram and figure creation tasks
   - Citation research and verification tasks

4. **Implementation**: Write content following task list
   - Draft prose following readability standards
   - Develop and test code examples
   - Create and validate equations
   - Generate or source diagrams
   - Add citations and references

5. **Review & Validation**: Verify content against specifications
   - Technical accuracy check
   - Citation verification
   - Code execution testing
   - Plagiarism detection
   - Readability assessment

6. **Documentation**: Record authoring sessions and decisions
   - Create Prompt History Records (PHRs) in `history/prompts/<chapter>/`
   - Document architectural decisions (ADRs) in `history/adr/` when significant choices are made

### Tooling Stack

- **Spec-Kit Plus**: Define and enforce chapter-level specifications
- **Claude Code**: Generate, revise, and validate structured technical content
- **Auxiliary LLMs (e.g., Qwen)**: Support numerical reasoning, simulations, control-system examples
- **Docusaurus**: Documentation framework for publishing the book as a live site
- **GitHub Pages**: Continuous integration and deployment
- **Git**: Version control with semantic versioning
- **VS Code / Claude Code**: Editing, version control, spec compliance
- **Plagiarism Detection Tools**: Run before final publication (e.g., Turnitin, Copyscape, or academic tools)

### Deliverables

- **Live Docusaurus site**: Navigable, searchable documentation with sidebar
- **PDF export**: Formatted manuscript with embedded citations and references
- **Optional EPUB**: For wider distribution
- **GitHub repository**: Full source with version history, specifications, and PHRs

## Governance

### Amendment Procedure

- **Proposal**: Any amendment to this constitution must be proposed with clear rationale
- **Documentation**: Proposed changes must be documented with expected impact on templates, workflow, and existing content
- **Approval**: Constitution changes require explicit user consent before implementation
- **Version increment**: Amendments must follow semantic versioning rules (MAJOR for breaking changes, MINOR for additions, PATCH for clarifications)
- **Propagation**: Template files and workflow documentation must be updated to reflect constitutional changes

### Versioning Policy

- **Semantic versioning**: Constitution versions follow MAJOR.MINOR.PATCH
  - MAJOR: Backward-incompatible principle removals or redefinitions
  - MINOR: New principles, sections, or material expansions
  - PATCH: Clarifications, wording improvements, non-semantic refinements
- **Tracking**: Version changes must be documented in the Sync Impact Report at the top of this file
- **Consistency**: All references to the constitution in templates and documentation must be updated when version changes

### Compliance Review

- **Pre-publication gate**: Manuscript must pass constitution compliance check before v1.0 release
- **Specification adherence**: All chapters must comply with their specifications
- **Citation audit**: Reference list must meet minimum peer-reviewed source requirements
- **Plagiarism scan**: Content must pass plagiarism detection before publication
- **Code validation**: All code examples must be tested and executable
- **Expert review**: Content must undergo peer review by domain experts in robotics, AI, and control theory

### Violation Handling

- **Constitution supersedes convenience**: When conflicts arise, constitutional principles take precedence
- **Justification required**: Any deviation from principles must be explicitly justified and documented
- **Remediation**: Violations discovered during review must be corrected before approval
- **Escalation**: Unresolved conflicts or ambiguities must be surfaced to the user for decision

### Architectural Decision Records (ADRs)

When architecturally significant decisions are made (framework choices, chapter structure, major tooling changes), an ADR must be suggested. ADRs are NEVER created automatically—user consent is required. Run `/sp.adr <decision-title>` when appropriate.

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19
