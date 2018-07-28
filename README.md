# barrier-certs-fw

Code for the following publications:

* [1] Squires, E., Pierpaoli, P., Konda, R., Coogan, S., & Egerstedt, M. (2022). Composition of safety constraints for fixed-wing collision avoidance amidst limited communications. Journal of Guidance, Control, and Dynamics, 45(4), 714-725.
* [2] Squires, E., Konda, R., Pierpaoli, P., Coogan, S., & Egerstedt, M. (2021, May). Safety with limited range sensing constraints for fixed wing aircraft. In 2021 IEEE International Conference on Robotics and Automation (ICRA) (pp. 9065-9071). IEEE.
* [3] Squires, E., Pierpaoli, P., & Egerstedt, M. (2018, August). Constructive barrier certificates with applications to fixed-wing aircraft collision avoidance. In 2018 IEEE Conference on Control Technology and Applications (CCTA) (pp. 1656-1661). IEEE.

Note this code uses the dynamics convention of [1] and [2] which have an
altitude component unlike [3].

## Build

Install [OSQP](https://osqp.org/). Publications above used sha `0f354d1894a3cfae36cf66c50d27e84c6427246d`

```bash
# see the scrimmage README for dependencies/if there are issues
git submodule update --init

mkdir ~/venvs
python3.6 -m venv ~/venvs/barrier-certs-fw
source ~/venvs/barrier-certs-fw/bin/activate
pip install -U wheel pip setuptools
pip install -r submodules/scrimmage/python/requirements-3.6.txt
pip install matplotlib

mkdir build
cd build
source /opt/scrimmage/*/setup.sh
cmake .. -DPYTHON_MIN_VERSION=3.6
make -j
source ~/.scrimmage-barrier-certs/setup.bash

cd -
cd submodules/scrimmage/python
pip install -e .

# make plots for aiaa/iros papers
# 1) see other choices with python scenarios.py -h
# 2) you can set enable_gui to false in barrier_certs.xml to not show the gui
# 3) this will generate more graphs than showed up in the publications which were space constrained
cd -
cd scripts
python scenarios.py aiaa ""
python scenarios.py icra ""

# note: you may need to reduce the dt in barrier_certs.xml
# if you find there are collisions (the algorithm assumes
# continuous time operation so if the dt is too large then
# there may be collisions)
```

see LICENSE for the license 
