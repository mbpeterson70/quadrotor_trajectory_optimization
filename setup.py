from setuptools import setup

setup(
    name='quad_traj_opt',
    version='0.1.0',    
    description='Quadrotor Trajectory Optimization',
    url='https://github.com/mbpeterson70/quadrotor_trajectory_optimization',
    author='Mason Peterson',
    author_email='masonbp@mit.edu',
    license='MIT',
    packages=['quad_traj_opt'],
    install_requires=['numpy',
                        'matplotlib',
                        'scipy',
                      ],
)