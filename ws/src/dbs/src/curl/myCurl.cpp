#include "curl/myCurl.hpp"

#include <curl/curl.h>
#include <iostream>
#include <fmt/core.h>


namespace curl {

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};


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


std::string push(std::string payload) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = payload;
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return request->query_response;
}

}