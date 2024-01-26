#!/bin/bash

USERNAME=root
PUBLIC_KEY=""
USER_ID=""
GROUP_ID=""

while [ "$#" -gt 0 ]; do
  case "$1" in
  -k | --key)
    PUBLIC_KEY=$2
    shift 2
    ;;
  -u | --user)
    USERNAME=$2
    shift 2
    ;;
  -i | --uid)
    USER_ID=$2
    shift 2
    ;;
  -g | --gid)
    GROUP_ID=$2
    shift 2
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done

# define the home directory of the specified user
HOME="/home/${USERNAME}"

# update the USER_ID and GROUP_ID of the specified user
if [ -n "${USER_ID}" ] && [ -n "${GROUP_ID}" ]; then
  groupmod -g "${GROUP_ID}" "${USERNAME}"
  usermod -u "${USER_ID}" -g "${GROUP_ID}" "${USERNAME}"
fi

if [ -n "${PUBLIC_KEY}" ]; then
  # authorise the specified user for ssh login
  mkdir -p "${HOME}"/.ssh
  echo "${PUBLIC_KEY}" >"${HOME}"/.ssh/authorized_keys
  chmod -R 755 "${HOME}"/.ssh
  chown -R "${USERNAME}:${USERNAME}" "${HOME}"/.ssh
fi

# save all environment variables to a file (except those listed below) and source it when the specified user logs in
env | egrep -v "^(HOME=|USER=|MAIL=|LC_ALL=|LS_COLORS=|LANG=|HOSTNAME=|PWD=|TERM=|SHLVL=|LANGUAGE=|_=)" >> /etc/environment
echo "source /etc/environment" | cat - "${HOME}/.bashrc" > tmp && mv tmp "${HOME}/.bashrc"

# start the ssh server
/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_development
