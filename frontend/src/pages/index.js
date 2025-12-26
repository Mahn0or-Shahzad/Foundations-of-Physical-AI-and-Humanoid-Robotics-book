import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="AI in Motion"
      description="Foundations of Physical AI and Humanoid Robotics - A comprehensive guide from ROS 2 fundamentals to autonomous humanoid systems">
      <main style={{
        maxWidth: '900px',
        margin: '0 auto',
        padding: '6rem 2rem',
        textAlign: 'center'
      }}>

        {/* Book Title */}
        <h1 className="homepage-title">
          AI in Motion: Foundations of Physical AI and Humanoid Robotics
        </h1>

        {/* Subtitle */}
        <p className="homepage-subtitle">
          A practical and academic guide to building autonomous humanoid systems—from ROS 2 fundamentals
          through digital twin simulation, GPU-accelerated perception, and vision-language-action integration.
        </p>

        {/* CTA Buttons */}
        <div className="homepage-buttons">
          <Link
            className="button button--primary button--lg"
            to="/docs/introduction">
            Start Reading
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/course-structure">
            Course Structure
          </Link>
        </div>

        {/* Key Topics */}
        <div className="homepage-modules">
          <div className="homepage-module-card">
            <h3 className="homepage-module-title">Module 1: ROS 2</h3>
            <p className="homepage-module-description">
              Distributed systems, nodes, topics, and URDF modeling
            </p>
          </div>
          <div className="homepage-module-card">
            <h3 className="homepage-module-title">Module 2: Digital Twins</h3>
            <p className="homepage-module-description">
              Gazebo, Unity, and Isaac Sim simulation environments
            </p>
          </div>
          <div className="homepage-module-card">
            <h3 className="homepage-module-title">Module 3: AI Perception</h3>
            <p className="homepage-module-description">
              GPU-accelerated SLAM, nvblox, and Nav2 navigation
            </p>
          </div>
          <div className="homepage-module-card">
            <h3 className="homepage-module-title">Module 4: VLA Systems</h3>
            <p className="homepage-module-description">
              Whisper, LLMs, CLIP for voice-controlled autonomy
            </p>
          </div>
          <div className="homepage-module-card">
            <h3 className="homepage-module-title">Capstone Project</h3>
            <p className="homepage-module-description">
              End-to-end autonomous humanoid integration
            </p>
          </div>
        </div>

      </main>
    </Layout>
  );
}
