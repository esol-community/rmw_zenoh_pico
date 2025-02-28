list of implemented rmw api on rmw_zenoh_pico (2025/02/28)

| RMW api                                         | status |
|-------------------------------------------------|--------|
| rmw_init                                        | OK     |
| rmw_shutdown                                    | OK     |
| rmw_init_options_fini                           | OK     |
| rmw_init_options_copy                           | OK     |
| rmw_init_options_init                           | OK     |
| rmw_create_subscription                         | OK     |
| rmw_destroy_subscription                        | OK     |
| rmw_subscription_count_matched_publishers       |        |
| rmw_subscription_get_actual_qos                 |        |
| rmw_subscription_set_content_filter             |        |
| rmw_subscription_get_content_filter             |        |
| rmw_create_node                                 | OK     |
| rmw_destroy_node                                | OK     |
| rmw_create_wait_set                             | OK     |
| rmw_destroy_wait_set                            | OK     |
| rmw_take                                        | OK     |
| rmw_take_with_info                              | OK     |
| rmw_wait                                        | OK     |
| rmw_create_publisher                            | OK     |
| rmw_destroy_publisher                           | OK     |
| rmw_publish                                     | OK     |
| rmw_publish_serialized_message                  |        |
| rmw_publish_loaned_message                      |        |
| rmw_publisher_wait_for_all_acked                |        |
| rmw_publisher_count_matched_subscriptions       |        |
| rmw_publisher_assert_liveliness                 |        |
| rmw_publisher_get_actual_qos                    |        |
| rmw_borrow_loaned_message                       |        |
| rmw_return_loaned_message_from_publisher        |        |
| rmw_return_loaned_message_from_subscription     | OK     |
| rmw_create_client                               | OK     |
| rmw_destroy_client                              | OK     |
| rmw_client_request_publisher_get_actual_qos     |        |
| rmw_client_response_subscription_get_actual_qos |        |
| rmw_client_set_on_new_response_callback         | OK     |
| rmw_service_set_on_new_request_callback         | OK     |
| rmw_send_request                                | OK     |
| rmw_take_request                                | OK     |
| rmw_take_sequence                               |        |
| rmw_take_event                                  | OK     |
| rmw_event_set_callback                          | OK     |
| rmw_publisher_event_init                        | OK     |
| rmw_subscription_event_init                     | OK     |
| rmw_take_serialized_message                     |        |
| rmw_take_serialized_message_with_info           |        |
| rmw_take_loaned_message_with_info               |        |
| rmw_take_response                               | OK     |

OK      : implemented (* is depend other futures)  
NO      : non support  
space   : no-plan yet  
