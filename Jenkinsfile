dockerNode(image: 'uf-mil:subjugator') {
	stage("Checkout") {
		checkout scm
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			git submodule update --init --recursive
			ln -s $PWD $CATKIN_DIR/src/SubjuGator
			git clone --recursive https://github.com/uf-mil/mil_common $CATKIN_DIR/src/mil_common
			ln -s $MIL_CONFIG_DIR/bvtsdk $CATKIN_DIR/src/mil_common/drivers/mil_blueview_driver/bvtsdk
		'''
	}
	stage("Build") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			catkin_make -C $CATKIN_DIR -B
		'''
	}
	stage("Test") {
		sh '''
			source ~/.mil/milrc > /dev/null 2>&1
			source $CATKIN_DIR/devel/setup.bash > /dev/null 2>&1
			catkin_make -C $CATKIN_DIR run_tests
			catkin_test_results $CATKIN_DIR/build/test_results --verbose 
		'''
	}
	stage("Format") {
		sh '''
			source /opt/ros/kinetic/setup.bash > /dev/null 2>&1
			wget -O ~/.clang-format https://raw.githubusercontent.com/uf-mil/installer/master/.clang-format
			for FILE in $(find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp'); do
				if [ ! -z "$(clang-format-3.8 -style=file -output-replacements-xml $FILE | grep '<replacement ')" ]; then
					FILES+=( "$FILE" )
				fi
			done
			if (( ${#FILES[@]} > 0 )); then
				echo "The C++ following files are not formatted correctly: ${FILES[@]}"
				exit 1
			fi
		'''
	}
}
