.PHONY: all
all: wrapper tracklib


wrapper:
	swig -c++ -python -threads tracker.i


tracklib:
	python build.py build_ext --inplace


.PHONY: clean
clean:
	rm -rf __pycache__/ build/ _Tracker.cpython-38-x86_64-linux-gnu.so Tracker.py tracker_wrap.cxx
