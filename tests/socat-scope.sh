#!/bin/bash
# runs a custom command in the scope of a socat session
# will automatically close socat when test ends

# set -e

cd $(dirname "${0}")

if [ -z "${1}" ]; then
    echo "No command provided, bail out"
    exit 1
fi

# run socat
socat pty,raw,echo=0,link=./fakeserialHMI pty,raw,echo=0,link=./fakeserialSYS &
socat_pid=${!}

# execute command
# set +e
${@} ./fakeserialSYS 1500000
ret=$?
# set -e

# terminate socat
kill -SIGTERM ${socat_pid}
wait ${socat_pid} || true

exit ${ret}
