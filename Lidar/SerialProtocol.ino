void sendPackage(uint8_t packageLength, uint8_t package[]) {
	Serial.write(packageLength);
	Serial.write(package, packageLength);
}