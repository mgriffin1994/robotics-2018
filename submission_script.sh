COMMIT=$1
LAB_NUM=$2

git diff $1 --name-only | xargs zip Lab"$LAB_NUM".zip
