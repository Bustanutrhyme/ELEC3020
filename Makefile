C99 = gcc
CFLAGS = -std=c11 -pedantic -Wall -Wextra

M_SRC = motor.c
S_SRC = servo.c
G_SRC = gps.c

MOTOR = motor
SERVO = servo
GPS	  = gps

all: build_gps build_motor build_servo

build_gps:
	@echo "Compile rakeserver"
	$(C99) $(CFLAGS) -o $(GPS) $(G_SRC)

build_motor:
	@echo "Compile rake-c"
	$(C99) $(CFLAGS) -o $(MOTOR) $(M_SRC)

build_servo:
	@echo "Compile rake-c"
	$(C99) $(CFLAGS) -o $(SERVO) $(S_SRC)

clean:
	rm -f $(MOTOR) $(SERVO) $(GPS)*.o