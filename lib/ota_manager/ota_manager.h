#pragma once

bool ota_begin(const char *hostname, const char *password);
void ota_handle();
bool ota_isUpdating();
bool ota_isAvailable();
