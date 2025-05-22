#include "curl/myCurl.hpp"

#include <curl/curl.h>
#include <iostream>
#include <fmt/core.h>

#include <nlohmann/json.hpp>


namespace curl {

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};


// static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
//     userp->append((char*)contents, size * nmemb);
//     return size * nmemb;
// }
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    size_t totalSize = size * nmemb;
    try {
        userp->append((char*)contents, totalSize);
    } catch (const std::bad_alloc&) {
        std::cerr << "Memory allocation failed." << std::endl;
        return 0;
    }
    return totalSize;
}

CURL *getCurlNeo4j(Request &request) {
    request.username   = "neo4j";
    request.password   = "123456789";
    request.url        = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return NULL;
    }

    const std::string auth = fmt::format("{}:{}", request.username, request.password);

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request.query_request.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(curl, CURLOPT_USERPWD, auth.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &(request.query_response));

    return curl;
}


CURL *getCurlInfluxDB_write(Request &request) {
    request.url = "http://172.17.0.1:8086/api/v2/write?org=TUBAF&bucket=STREAM&precision=ns";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return NULL;
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: text/plain");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());

    return curl;
}

CURL *getCurlInfluxDB_read(Request &request) {
    request.url = "http://172.17.0.1:8086/api/v2/query?org=TUBAF&bucket=STREAM&precision=ns";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return NULL;
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/vnd.flux");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);
    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());

    return curl;
}

std::string push(std::string payload, const Destination destination) {
    struct Request request;

    CURL *curl;// = ((destination == NEO4J) ? getCurlNeo4j(request) : getCurlInfluxDB(request));
    switch (destination)
    {
    case NEO4J:
        curl = getCurlNeo4j(request);
        break;
    case INFLUXDB_WRITE:
        curl = getCurlInfluxDB_write(request);
        break;
    case INFLUXDB_READ:
        curl = getCurlInfluxDB_read(request);
        break;
    default:
        std::cerr << "unknown request" << std::endl;
        break;
    }

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cout << payload << std::endl;
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        if (destination == NEO4J) {
            nlohmann::json j = request.query_response;
            if (j.contains("errors") && !j["errors"].empty()) {
                std::cout << request.query_response << std::endl;
                std::string code = j["errors"][0]["code"];
                std::string message = j["errors"][0]["message"];
                std::cout << "Error Code: " << code << std::endl;
                std::cout << "Error Message: " << message << std::endl;
            }
        }
        // TODO add errorHandling for InfluxDB
    }

    return request.query_response;
}

}