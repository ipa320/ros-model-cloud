#/bin/bash
git clone $1

python mock/test.py --package $2 --name $3 --node >> $3.ros
