find . -name __pycache__ -exec rm -rf {} \;
find . -name build -exec rm -rf {} \;
find . -name dist -exec rm -rf {} \;
find . -name *.egg-info -exec rm -rf {} \;
