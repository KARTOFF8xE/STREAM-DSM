#pragma once

#include <fmt/core.h>
#include <string>
#include <iostream>

#include "../interface.h"
#include "../curl.cpp"

class FooNode: public IParticipant {
    private:
        std::string name;
        const char *nameSpace;
        u_int64_t   handle;
        u_int32_t   pid;

        std::string getPayload() override {
            return fmt::format(R"(
                {{
                    "statements":
                        [
                            {{ "statement": "CREATE (:Node{{name:$name, namespace:$namespace, handle:$handle, pid:$pid, state:1}}) ",
                            "parameters": {{
                                "name": "{}",
                                "namespace": "{}",
                                "handle": "{}",
                                "pid": "{}"
                                }}
                            }}
                        ]
                }}
            )", this->name, this->nameSpace, this->handle, this->pid);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "namespace");
                    this->nameSpace = bt_field_string_get_value(field);
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->handle = bt_field_integer_unsigned_get_value(field);
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

            const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
            field_class = bt_field_borrow_class_const(ctx_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(ctx_field, "vpid");
                this->pid = uint32_t(bt_field_integer_signed_get_value(field));
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

            return;
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }

};