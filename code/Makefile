
CXX?=g++
EXTRAARGS?=
CFLAGS=-I. -D__EXPORT="" -Dnullptr="0" -g -fno-strict-aliasing \
			   -fno-strength-reduce \
			   -fomit-frame-pointer \
   			   -funsafe-math-optimizations \
   			   -Wall \
			   -Wextra \
			   -Wshadow \
			   -Wfloat-equal \
			   -Wpointer-arith \
			   -Wpacked \
			   -Wno-unused-parameter \
			   -Werror=format-security \
			   -Wmissing-field-initializers \
			   -Wfatal-errors \
			   -Wno-missing-field-initializers \
			   -std=c++11

ODIR=obj
LDIR =../lib

LIBS=-lm

#_DEPS = test.h
#DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = main_closed_loop_float.o estimator_22states.o estimator_utilities.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

#$(DEPS)
$(ODIR)/%.o: %.cpp
	mkdir -p $(ODIR)
	$(CXX) -c -o $@ $< $(CFLAGS)

#
estimator_closed_loop_test: $(OBJ)
	$(CXX) -g -o $@ $^ $(CFLAGS) $(LIBS) $(EXTRAARGS)

.PHONY: clean plots

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

plots:
	python3 plot_attitude.py &
#	python3 plot_position.py &
	python3 plot_states.py &
#	python3 plot_velocity.py &
#	python3 plot_flow.py &
	python3 plot_glitchOffset.py &

tests:
	make clean
	export CXX="g++"
	export EXTRAARGS=""
	make
	unzip -o InputFilesPX4.zip
	./estimator_closed_loop_test
#make clean
#export CXX=arm-none-eabi-gcc
#export EXTRAARGS=--specs=nosys.specs
#make
#export CXX="g++"
#export EXTRAARGS=""
	make clean
	./validate_output.py
