#############################################################################################
# Build Documentation
#############################################################################################

help:
	@echo "Documentation"
	@echo "  docs      : buidl sphinx documentation"

docs:
	PY_TREES_DISABLE_COLORS=1 sphinx-build -E -b html doc doc/html

clean:
	-rm -rf doc/html

.PHONY: docs clean
