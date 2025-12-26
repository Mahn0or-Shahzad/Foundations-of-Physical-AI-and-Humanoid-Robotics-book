import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Foundations',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master the Robot Operating System 2 middleware - the nervous system for
        humanoid robots. Learn nodes, topics, services, and actions.
      </>
    ),
  },
  {
    title: 'Digital Twin Simulation',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Create physics-accurate simulations using Gazebo, Unity, and NVIDIA Isaac Sim.
        Build and test algorithms safely before deploying to hardware.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action AI',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integrate large language models with robotic perception and control.
        Build autonomous humanoids that understand voice commands and execute tasks.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
