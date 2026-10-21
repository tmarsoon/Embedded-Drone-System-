MVN ?= mvn
JOBS ?= 4
.PHONY: all native java test run research sanitize
all: native java
native:
	$(MAKE) -C native -j$(JOBS) all
java:
	$(MVN) -B -C -DskipTests package
test: native
	$(MAKE) -C native -j$(JOBS) test
	$(MVN) -B -C test
run: all
	$(MVN) -C javafx:run
research: native
	python3 native/tools/research.py --binary native/build/drone_sim --output runs/research --runs 30
sanitize:
	$(MAKE) -C native -j$(JOBS) sanitize
