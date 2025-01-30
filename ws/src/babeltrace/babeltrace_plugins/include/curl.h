#pragma once

#include <curl/curl.h>

#include "interface.h"

// size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);

CURL *getCurl(Request *request);