.PHONY: all
all: wrapper tracklib


wrapper:
	swig -c++ -python -threads tracker.i


tracklib:
	python3 build.py build_ext --inplace


.PHONY: clean
clean:
	rm -rf __pycache__/ build/ _Tracker*.so Tracker.py tracker_wrap.cxx
