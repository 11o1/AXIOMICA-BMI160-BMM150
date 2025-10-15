#!/bin/sh
echo "$(date +"%Y-%m-%d %H:%M:%S") $1"
git config http.postBuffer 524288000
git add *
git commit -am "$(date +"%Y-%m-%d %H:%M:%S") $1"
git push
