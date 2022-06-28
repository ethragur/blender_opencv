# SPDX-License-Identifier: GPL-2.0-or-later

ExternalProject_Add(external_libepoxy
  URL file://${PACKAGE_DIR}/${LIBEPOXY_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${LIBEPOXY_HASH_TYPE}=${LIBEPOXY_HASH}
  PREFIX ${BUILD_DIR}/libepoxy
  CONFIGURE_COMMAND meson setup --prefix ${LIBDIR}/libepoxy --default-library static --libdir lib . ../external_libepoxy -Dtests=false
  BUILD_COMMAND ninja
  INSTALL_COMMAND ninja install
)
