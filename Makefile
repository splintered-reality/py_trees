#############################################################################################
# Build Documentation
#############################################################################################

help:
	@echo "Documentation"
	@echo "  docs      : buidl sphinx documentation"

docs:
	PY_TREES_DISABLE_COLORS=1 sphinx-build -E -b html docs docs/html

clean:
	-rm -rf docs/html

.PHONY: docs clean
