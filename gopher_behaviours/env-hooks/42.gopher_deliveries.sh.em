
# variables

# The default custom delivery locations come from the default desirable_destinations.yaml list loaded by
# gopher_bootstrap env hooks.
: ${GOPHER_CUSTOM_DELIVERY_LOCATIONS:="pizza_shop, ashokas_hell, anywhere_not_near_the_wife"}
: ${GOPHER_CUSTOM_DELIVERY_LOCATIONS_DESC:="list of locations to deliver to when no delivery is active and the go button is pressed"}

# exports
export GOPHER_CUSTOM_DELIVERY_LOCATIONS
export GOPHER_CUSTOM_DELIVERY_LOCATIONS_DESC
