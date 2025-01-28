#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fmt/core.h>

// Funktion, um die Antwort von Neo4j zu speichern
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

int main() {
    // URL der Query API
    const std::string neo4j_url = "http://172.17.0.1:7474/db/neo4j/tx/commit"; // $ ip addr show docker0 for IP-address
    const std::string username = "neo4j";
    const std::string password = "123456789";

    // Cypher-Abfrage
    std::string cypher_query = fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "CREATE (n:Node{{name:\"{}\"}}) " }}
                ]
        }}
    )", "BAR");
// std::string cypher_query = R"(
// {
//     "statements":
//         [
//             { "statement": "CREATE (n:Node{name:\"BAR\"}) " }
//         ]
// }
// )";

// std::string cypher_query = R"(
//     { "statements": [ { "statement": "MATCH (n) RETURN n"} ] }
// )";

    std::cout << cypher_query << std::endl;

    // Libcurl-Initialisierung
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return 1;
    }

    // Speicher für die Antwort
    std::string response_string;
    std::string header_string;

    // Authentifizierungsdaten (Base64-codiert)
    const std::string auth = username + ":" + password;

    // Libcurl-Einstellungen
    curl_easy_setopt(curl, CURLOPT_URL, neo4j_url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, cypher_query.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(curl, CURLOPT_USERPWD, auth.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

    // Setze die Header (Content-Type: application/json)
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // Anfrage senden
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // Ausgabe der Antwort
        std::cout << "Antwort von Neo4j:" << std::endl;
        std::cout << response_string << std::endl;
    }

    // Speicher freigeben
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return 0;
}