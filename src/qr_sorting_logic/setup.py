from setuptools import setup

package_name = 'qr_sorting_logic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MAI605 Student',
    maintainer_email='student@example.com',
    description='QR decision logic for QR-based robot sorting.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qr_decision_node = qr_sorting_logic.qr_decision_node:main',
            'qr_test_injector = qr_sorting_logic.qr_test_injector:main',
        ],
    },
)
