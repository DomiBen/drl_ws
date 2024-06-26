#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/domi/drl_ws/src/hrl-kdl/hrl_geom"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/domi/drl_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/domi/drl_ws/install/lib/python3/dist-packages:/home/domi/drl_ws/build/hrl_geom/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/domi/drl_ws/build/hrl_geom" \
    "/usr/bin/python3" \
    "/home/domi/drl_ws/src/hrl-kdl/hrl_geom/setup.py" \
     \
    build --build-base "/home/domi/drl_ws/build/hrl_geom" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/domi/drl_ws/install" --install-scripts="/home/domi/drl_ws/install/bin"
