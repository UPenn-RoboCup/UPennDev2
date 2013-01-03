for file in `ls *.m`
do
	cat copyright.txt $file > tmp
  mv tmp $file
  echo "Writing $file ..."
done
