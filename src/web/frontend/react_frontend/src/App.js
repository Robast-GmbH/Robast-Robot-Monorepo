import Header from './components/Header.js'
import { BrowserRouter as Router, Route, Switch } from 'react-router-dom'
import { useState, useEffect } from 'react'
import AddOrder from './components/AddOrder.js'
import Footer from './components/Footer.js'
import About from './components/About.js'
import ShowOrders from './components/ShowOrders.js'
import React from 'react'
import ControlSwitch from './components/ControlSwitch.js'
import CallRobotPosition from './components/CallRobotPosition.js'
import MapPositions from './components/MapPositions.js'
import SendGoal from './components/SendGoal.js'
import Button from './components/Button.js'
import AddMapPosition from './components/AddMapPosition.js'
import Popup from 'reactjs-popup';


const backend_address = `http://localhost:8000`
const user_id = 1;
function App() {
  const [showAddOrder, setShowAddOrder] = useState(false)
  const [showOrder, setShowOrder] = useState(false)
  const [popupIsOpen, setPopupIsOpen] = useState(false)
  const [orders, setOrders] = useState([])
  const [mapPositions, setMapPositions] = useState([
    { 
      id: 1,
      title: 'example position',
      Coordinates: {x:0.0, y: 0.0}
    }
  ])
  const [drawers, setDrawers]= useState([])

  const [OrderCoords, setOrderCoords] = useState({
    Coords:{x:0, y:0, scale_x:0, scale_y:0}})

  const [User, setUser] = useState({
      Metadata:{id:0, email: "", name:""}})

  const closeModal = () => setPopupIsOpen(false);
  const openModal = () => setPopupIsOpen(true);

  useEffect(() => {
    const getOrders = async () => {
      const ordersFromServer = await fetchOrders()
      const drawersFromServer =await fetchDrawersByRobot(1)
      console.log(ordersFromServer)
      setOrders(ordersFromServer)
      setDrawers(drawersFromServer)
    }

    getOrders()
  }, [])

  // useEffect(() => {
  //   const getMapPositions = async () => {
  //     const mapPositionsFromServer = await fetchMapPositions()
  //     console.log(mapPositionsFromServer)
  //     setMapPositions(mapPositionsFromServer)
  //   }

  //   getMapPositions()
  // }, [])
  
  // useEffect(() => {
  //   const getGoals = async () => {
  //     const mapPositionsFromServer = await fetchMapPositions()
  //     console.log(mapPositionsFromServer)
  //     setMapPositions(mapPositionsFromServer)
  //   }

  //   getGoals()
  // }, [])

 

  const fetchUser = async () => {
    const res = await fetch(`${backend_address}/users/${user_id}`)
    const data = await res.json()

    return data;
  }

  const fetchOrders = async () => {
    const res = await fetch(`${backend_address}/orders`)
    const data = await res.json()

    return data;
  }
  const fetchMapPositions = async () => {
    const res = await fetch(`${backend_address}/mappositions`)
    const data = await res.json()

    return data;
  }

  const fetchOrder = async (id) => {
    const res = await fetch(`${backend_address}/orders/${id}`)
    const data = await res.json()

    return data;
  }
 

  //DELETE
  const deleteOrder = async (id) => {
    await fetch(`${backend_address}/orders/${id}`, { method: 'DELETE' })

    setOrders(orders.filter((order) => order.id !== id));
  }

  const toggleRecurringOrder = async (order_id) => {
    const orderToToggle = await fetchOrder(order_id)
    const updatedOrder = { ...orderToToggle, recurring_order: !orderToToggle.recurring_order }
    

    const res = await fetch(`${backend_address}/orders/${order_id}`, {
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
    const res = await fetch(`${backend_address}/users/${user_id}/order`, {
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

  const addMapPosition = async (mapPosition) => {
    console.log(mapPosition)
    const res = await fetch(`${backend_address}/users/${user_id}/mapposition`, {
      method: 'POST',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(mapPosition)
    })
    const data = await res.json()
    console.log(data)
    setMapPositions([...mapPositions, data])
    closeModal()
  }
  
  //Drawer
  const fetchDrawersByRobot = async (robot_id) => {
    const res = await fetch(`${backend_address}/orders/drawers/${robot_id}`)
    const data = await res.json()

    return data;
  }

  const fetchDrawer = async (robot_id, index) => {
    const res = await fetch(`${backend_address}/drawers/${robot_id}/${index}`)
    const data = await res.json()

    return data;
  }

  const updateDrawer = async (drawer) => {
    console.log(drawer)
    const res = await fetch(`${backend_address}/drawers/1/`, {
      method: 'POST',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(drawer)
    })
    const data = await res.json()
    console.log(data)
    setDrawers(fetchDrawersByRobot())
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
          labelOpen={"Add Position"}
          onAdd={() => setShowOrder(!showOrder)}
          showAdd={showOrder} />

        {showOrder && <AddMapPosition onAdd={addMapPosition}/>}
            {mapPositions.length > 0 ? (
              <MapPositions mapPositions={mapPositions}/>
              ):(
                'No map positions set yet'
              )
        }
        {/* <MapPositions mapPositions={mapPositions} sendGoal={SendGoal}></MapPositions>
        <Button text={'Add Map Position'} ></Button> */}
        <Switch>

          <Route path='/' elements={
            <>
            {<AddMapPosition onAdd={addMapPosition}/>}
            {mapPositions.length > 0 ? (
              <MapPositions mapPositions={mapPositions}/>
              ):(
                'No map positions set yet'
                )}
            </>

          } />
        
          <Route path='/about' component={About}></Route>
        </Switch>
        <Footer />
      </div>
    </Router>
  );
}

export default App;


// get json from apenapi: https://www.npmjs.com/package/openapi-typescript

/*
<RosMap onClick={showCoords} />
            <Popup open={popupIsOpen} closeOnDocumentClick onClose={closeModal}>
              <>
                {<AddOrder onAdd={addOrder} coords={OrderCoords} />}
              </>
            </Popup>
*/