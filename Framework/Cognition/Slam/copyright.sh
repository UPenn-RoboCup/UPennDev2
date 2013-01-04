for file in `ls *.c* *.h*`
do
	cat copyright.txt $file > tmp
  mv tmp $file
  echo "Writing $file ..."
done
