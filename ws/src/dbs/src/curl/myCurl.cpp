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

// Hilfsstruktur, um CURL* und Header-Liste gemeinsam zu verwalten
struct CurlHandleWithHeaders {
    CURL* curl;
    struct curl_slist* headers;
};

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

// Hilfsfunktion: Führt Request aus, gibt Header und CURL frei
static CURLcode performAndCleanup(CurlHandleWithHeaders& handle) {
    CURLcode res = curl_easy_perform(handle.curl);
    if (handle.headers) {
        curl_slist_free_all(handle.headers); // Header-Liste freigeben
    }
    curl_easy_cleanup(handle.curl); // CURL-Handle freigeben
    return res;
}

CurlHandleWithHeaders getCurlNeo4j(Request &request) {
    request.username   = "neo4j";
    request.password   = "123456789";
    request.url        = "http://neo4j:7474/db/neo4j/tx/commit";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return {nullptr, nullptr};
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

    return {curl, headers};
}

CurlHandleWithHeaders getCurlInfluxDB_write(Request &request) {
    request.url = "http://influxdb:8086/api/v2/write?org=TUBAF&bucket=STREAM&precision=ns";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return {nullptr, nullptr};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: text/plain");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);
    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());

    return {curl, headers};
}

CurlHandleWithHeaders getCurlInfluxDB_read(Request &request) {
    request.url = "http://influxdb:8086/api/v2/query?org=TUBAF&bucket=STREAM&precision=ns";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return {nullptr, nullptr};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/vnd.flux");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);
    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());

    return {curl, headers};
}

CurlHandleWithHeaders getCurlInfluxDB_setTask(Request &request) {
    request.url = "http://influxdb:8086/api/v2/tasks";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return {nullptr, nullptr};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, request.url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);

    return {curl, headers};
}

CurlHandleWithHeaders getCurlInfluxDB_updateTask(Request &request, const std::string& taskId) {
    const std::string url = "http://influxdb:8086/api/v2/tasks/" + taskId;
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Curl Init Fehler" << std::endl;
        return {nullptr, nullptr};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);

    return {curl, headers};
}

CurlHandleWithHeaders getCurlInfluxDB_deleteTask(Request& request, const std::string& taskId) {
    const std::string url = "http://influxdb:8086/api/v2/tasks/" + taskId;
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Curl Init Fehler" << std::endl;
        return {nullptr, nullptr};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &request.query_response);

    return {curl, headers};
}

std::string getTaskIdsForNames() {
    const std::string url = "http://influxdb:8086/api/v2/tasks?org=TUBAF";
    const std::string token = "WVvSEEbHPqeMFpcgWqThaEcU6u6SWJ-L26ct4oRuEJmKdMOk-ZG8XlKA5xcitJXENa2r2YNLNwxjE6-KKkx8xw==";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Curl Init Fehler" << std::endl;
        return "";
    }

    std::string response;
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, ("Authorization: Token " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(headers); // <--- Wichtig: Header-Liste freigeben
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        std::cerr << "CURL Fehler: " << curl_easy_strerror(res) << std::endl;
        return "";
    }

    return response;
}

// --- Push-Funktionen bleiben wie gehabt ---

std::string push(std::string payload, const Destination destination) {
    struct Request request;
    CurlHandleWithHeaders handle{nullptr, nullptr};

    switch (destination)
    {
    case NEO4J:
        handle = getCurlNeo4j(request);
        break;
    case INFLUXDB_WRITE:
        handle = getCurlInfluxDB_write(request);
        break;
    case INFLUXDB_READ:
        handle = getCurlInfluxDB_read(request);
        break;
    case INFLUXDB_SETTASK:
        handle = getCurlInfluxDB_setTask(request);
        break;
    default:
        std::cerr << "unknown request" << std::endl;
        return "";
    }

    if (!handle.curl) {
        if (handle.headers) curl_slist_free_all(handle.headers);
        return "";
    }

    curl_easy_setopt(handle.curl, CURLOPT_POSTFIELDS, payload.c_str());
    CURLcode res = performAndCleanup(handle);

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

std::string push(std::string payload, const Destination destination, std::string taskId) {
    struct Request request;
    CurlHandleWithHeaders handle{nullptr, nullptr};

    switch (destination) {
    case INFLUXDB_UPDATETASK:
        handle = getCurlInfluxDB_updateTask(request, taskId);
        break;
    default:
        std::cerr << "unknown request" << std::endl;
        return "";
    }

    if (!handle.curl) {
        if (handle.headers) curl_slist_free_all(handle.headers);
        return "";
    }

    curl_easy_setopt(handle.curl, CURLOPT_POSTFIELDS, payload.c_str());
    CURLcode res = performAndCleanup(handle);

    if (res != CURLE_OK) {
        std::cout << payload << std::endl;
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // TODO add errorHandling for InfluxDB
    }

    return request.query_response;
}

std::string push(const Destination destination, std::string taskId) {
    struct Request request;
    CurlHandleWithHeaders handle{nullptr, nullptr};

    switch (destination) {
    case INFLUXDB_DELETETASK:
        handle = getCurlInfluxDB_deleteTask(request, taskId);
        break;
    default:
        std::cerr << "unknown request" << std::endl;
        return "";
    }

    if (!handle.curl) {
        if (handle.headers) curl_slist_free_all(handle.headers);
        return "";
    }

    curl_easy_setopt(handle.curl, CURLOPT_POSTFIELDS, "");
    CURLcode res = performAndCleanup(handle);

    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // TODO add errorHandling for InfluxDB
    }

    return request.query_response;
}

}