if [ -z $1 ]; then
  echo "specify a lab to zip"
  exit
fi

lab=$1

rm $lab.zip

cd $lab
zip -r9 ../$lab *
cd ..

cd lib
zip -r9 ../$lab **/*.py
cd ..
