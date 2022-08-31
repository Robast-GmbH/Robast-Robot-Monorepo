import Header from './components/Header.js'
import { BrowserRouter as Router, Route } from 'react-router-dom'
import { useState, useEffect } from 'react'
import AddOrder from './components/AddOrder.js'
import Footer from './components/Footer.js'
import About from './components/About.js'
import Popup from 'reactjs-popup';
import RosMap from './components/RosMap.js'
import ShowOrders from './components/ShowOrders.js'
import React from 'react'

const user_id = 1;
function App() {
  const [showAddOrder, setShowAddOrder] = useState(false)
  const [showOrder, setShowOrder] = useState(false)
  const [popupIsOpen, setPopupIsOpen] = useState(false)
  const [orders, setOrders] = useState([

  ])
  const [OrderCoords, setOrderCoords] = useState({
    Coords:{x:0, y:0, scale_x:0, scale_y:0}})

  const [User, setUser] = useState({
      Metadata:{id:0, email: "", name:""}})

  const closeModal = () => setPopupIsOpen(false);
  const openModal = () => setPopupIsOpen(true);

  useEffect(() => {
    const getOrders = async () => {
      const ordersFromServer = await fetchOrders()
      console.log(ordersFromServer)
      setOrders(ordersFromServer)
    }

    getOrders()
  }, [])

  const fetchUser = async () => {
    const res = await fetch(`http://localhost:8000/users/${user_id}`)
    const data = await res.json()

    return data;
  }

  const fetchOrders = async () => {
    const res = await fetch(`http://localhost:8000/orders`)
    const data = await res.json()

    return data;
  }

  const fetchOrder = async (id) => {
    const res = await fetch(`http://localhost:8000/orders/${id}`)
    const data = await res.json()

    return data;
  }

  //DELETE
  const deleteOrder = async (id) => {
    await fetch(`http://localhost:8000/orders/${id}`, { method: 'DELETE' })

    setOrders(orders.filter((order) => order.id !== id));
  }

  const toggleRecurringOrder = async (order_id) => {
    const orderToToggle = await fetchOrder(order_id)
    const updatedOrder = { ...orderToToggle, recurring_order: !orderToToggle.recurring_order }
    

    const res = await fetch(`http://localhost:8000/orders/${order_id}`, {
      method: 'PUT',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(updatedOrder)
    })

    const data = await res.json() 
    console.log("log")   
    console.log(res)   
    setOrders(
      orders.map((order) =>        
        order.id === order_id ? { ...order, recurring_order: data.recurring_order } : order))
  }

  const addOrder = async (order) => {
    console.log(order)
    const res = await fetch(`http://localhost:8000/users/${user_id}/order`, {
      method: 'POST',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(order)
    })
    const data = await res.json()
    console.log(data)
    setOrders([...orders, data])
    closeModal()
  }

  const showCoords = (event) => {

    openModal();
    const rect = document.getElementById('RosMap').getBoundingClientRect();
    const map_x = event.clientX - rect.left;
    const map_y = event.clientY - rect.top;   
    
    setOrderCoords({x:map_x, y:map_y, clientX:1500, clientY:500});
    console.log(rect);
    console.log(OrderCoords);
    console.log("map_x: " + map_x + ", map_y:" + map_y);
  }

  return (

    <Router>
      
      <div className='container' id="Map">
        <Header
          labelOpen={"Show Orders"}
          onAdd={() => setShowOrder(!showOrder)}
          showAdd={showOrder} />

        <Route path='/' exact render={(props) => (
          <>
            {showOrder  && <ShowOrders orders={orders} onDelete={deleteOrder} onToggle={toggleRecurringOrder}></ShowOrders>}
            {<h1>click on the location to order too</h1>}

            <RosMap onClick={showCoords} />
            <Popup open={popupIsOpen} closeOnDocumentClick onClose={closeModal}>
              <>
                {<AddOrder onAdd={addOrder} coords={OrderCoords} />}
              </>
            </Popup>
          </>
        )} />
        <Route path='/about' component={About}></Route>
        <Footer />

      </div>
    </Router>
  );
}

export default App;


// get json from apenapi: https://www.npmjs.com/package/openapi-typescript