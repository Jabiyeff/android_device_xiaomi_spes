# ROM source patches

color="\033[0;32m"
end="\033[0m"

echo -e "${color}Applying patches${end}"
sleep 1

DEVICE_PATH="device/xiaomi/spes"

# ROM codename check
rom_check=$(grep -n "LOCAL_DIR" ${DEVICE_PATH}/AndroidProducts.mk | grep -Eo '^[^:]+')
array=(${rom_check})
rom_name=$(sed -n ${array[0]}'p' < ${DEVICE_PATH}/AndroidProducts.mk | cut -d "/" -f 2 | tr -d '[:space:]' | cut -d "_" -f 1)

ROM_VENDOR_PATH="vendor/${rom_name}"

# Patch status check
patch_check=$(grep -c "TARGET_USES_CUSTOM_DISPLAY_INTERFACE" ${ROM_VENDOR_PATH}/config/BoardConfigQcom.mk)

if [ ${patch_check} -eq 0 ]
then
cd ${ROM_VENDOR_PATH}
git am ../../${DEVICE_PATH}/patches/custom-display-commonsys-intf.patch
cd ../..
else
echo -e "${color}Source patches already merged${end}"
fi
sleep 1

# Remove pixel headers to avoid conflicts
rm -rf hardware/google/pixel/kernel_headers/Android.bp
