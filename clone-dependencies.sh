dependencies=$(rospack depends $1) # rospack depends1 for immediate dependencies?

for d in $dependencies
do
	info=$(roslocate info $d)
	tmp=${info#*uri: }
	repo=${tmp%version*}
	git clone $repo
done
