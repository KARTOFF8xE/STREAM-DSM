#pragma once

#include <iostream>
#include <fmt/core.h>
#include <curl/curl.h>

#include "interface.h"

// store response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

CURL *getCurl(Request *request) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return NULL;
    }

    const std::string auth = fmt::format("{}:{}", request->username, request->password);

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_URL, request->url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request->query_request.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(curl, CURLOPT_USERPWD, auth.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &(request->query_response));

    return curl;
}