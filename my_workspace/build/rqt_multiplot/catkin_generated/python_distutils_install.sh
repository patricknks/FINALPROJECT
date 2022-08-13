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

echo_and_run cd "/home/patricknks/FINALPROJECT/my_workspace/src/rqt_multiplot_plugin"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/patricknks/FINALPROJECT/my_workspace/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/patricknks/FINALPROJECT/my_workspace/install/lib/python2.7/dist-packages:/home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot" \
    "/usr/bin/python2" \
    "/home/patricknks/FINALPROJECT/my_workspace/src/rqt_multiplot_plugin/setup.py" \
    egg_info --egg-base /home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot \
    build --build-base "/home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/patricknks/FINALPROJECT/my_workspace/install" --install-scripts="/home/patricknks/FINALPROJECT/my_workspace/install/bin"
