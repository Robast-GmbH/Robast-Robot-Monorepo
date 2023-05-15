from unittest.mock import Mock
from fastapi.testclient import TestClient
from fastapi import FastAPI
from web_bridge_nfc.rest_interface import RestInterface
# from web_bridge_nfc.ros_controller import ros_controller

# Test setup
ROS_node_mock = Mock(name="ros")

app = FastAPI()
Test_class = RestInterface(ROS_node_mock, app)
client = TestClient(app)


def test_create_new_user():
    expected_reader_status = "Card_required"
    ROS_node_mock.reader_in_use = False
    ROS_node_mock.reader_status = expected_reader_status
    ROS_node_mock.id = 3

    responce = client.post("/users/", json={"first_name": "Karl", "last_name": "Toffel"})
    assert(responce.is_success)


def test_create_new_user_reader_in_use():
    ROS_node_mock.reader_in_use = True
    responce = client.post("/users/", json={"first_name": "Karl", "last_name": "Toffel"})
    assert(responce.is_server_error)


def test_create_new_user_only_first_name():
    ROS_node_mock.reader_in_use = False
    responce = client.post("/users/", json={"first_name": "Karl"})
    assert(responce.is_client_error)


def test_create_new_user_only_last_name():
    ROS_node_mock.reader_in_use = False
    responce = client.post("/users/", json={"first_name": "Karl"})
    assert(responce.is_client_error)


def test_create_new_card():
    valid_id = "1234"
    ROS_node_mock.reader_in_use = False
    ROS_node_mock.call_create_nfc_tag_action
    ROS_node_mock.ros_node.id = valid_id
    responce = client.post("/users/nfc/", json={"id": valid_id})
    assert(responce.is_success)


def test_create_new_card_no_id():
    ROS_node_mock.reader_in_use = False
    responce = client.post("/users/nfc/")
    assert(responce.is_client_error)


def test_create_new_card_reader_in_use():
    ROS_node_mock.reader_in_use = True
    responce = client.post("/users/nfc/", json={"id": "66"})
    assert(responce.is_server_error)


def test_check_status_of_existing_task():
    expected_reader_status = "Card_required"
    ROS_node_mock.reader_in_use = True
    ROS_node_mock.reader_status = expected_reader_status
    ROS_node_mock.id = 3

    responce = client.get("/users/nfc/3")
    assert(responce.json() == {'Status': expected_reader_status})
    assert(responce.is_success)


def test_check_status_of_not_existing_task():
    ROS_node_mock.reader_in_use = True
    ROS_node_mock.id = 2

    responce = client.get("/users/nfc/3")
    assert(responce.status_code == 404)


def test_check_status_of_task_no_aktive_task():
    ROS_node_mock.reader_in_use = False

    responce = client.get("/users/nfc/3")
    assert(responce.status_code == 404)
