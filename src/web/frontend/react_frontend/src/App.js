import Header from './components/Header.js'
import { BrowserRouter as Router, Route, Switch } from 'react-router-dom'
import { useState, useEffect,useRef } from 'react'
import Footer from './components/Footer.js'
import About from './components/About.js'
import React from 'react'
import ControlSwitch from './components/ControlSwitch.js'
import RobotControl from './components/RobotControl.js'
import DrawerControl from './components/DrawerControl.js'



const backend_address = `http://localhost:8000`
const user_id = 1;
function App() {
  //const [showAddOrder, setShowAddOrder] = useState(false)
  //const [showOrder, setShowOrder] = useState(false)
  //const [popupIsOpen, setPopupIsOpen] = useState(false)
  //const [AddPositionIsOpen, setAddPositionIsOpen]=useState(false)
  //const [orders, setOrders] = useState([])
  const [mapPositions, setMapPositions] = useState([])
  const [drawers, setDrawers]= useState([])


  //const [OrderCoords, setOrderCoords] = useState({
  //  Coords:{x:0, y:0, scale_x:0, scale_y:0}})

  //const [User, setUser] = useState({
  //    Metadata:{id:0, email: "", name:""}})

  //const closeModal = () => setPopupIsOpen(false);
  //const openModal = () => setPopupIsOpen(true);
  //const closePositionPppUp= () => setShowOrder(false);


   useEffect(() => {
     const getMapPositions = async () => {
      const mapPositionsFromServer = await fetchMapPositions()
     //console.log(mapPositionsFromServer)
      setMapPositions(mapPositionsFromServer)
    }

     getMapPositions()
   }, [])

   useEffect(() => {
    
    updateDrawersFromDB()
    setInterval(() => { updateDrawersFromDB()},30000);
  }, [])
  
  
  
  // const fetchUser = async () => {
  //   const res = await fetch(`${backend_address}/users/${user_id}`)
  //   const data = await res.json()

  //   return data;
  // }
  const updateDrawersFromDB= async () => {
    const drawersFromServer =await fetchDrawers()
    //console.log({drawersFromServer})
    setDrawers(drawersFromServer)
 }
  
  const fetchMapPositions = async () => {
    const res = await fetch(`${backend_address}/map_positions`)
    const data = await res.json()
    return data;
  }

  const addMapPosition = async (mapPosition) => {
    console.log(mapPosition)
    const res = await fetch(`${backend_address}/map_positions`, {
      method: 'POST',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(mapPosition)
    })
    const data = await res.json()
    console.log(data)
    setMapPositions([...mapPositions, data])
  }
  
  //Drawer
  const fetchDrawers = async () => {
    const res = await fetch(`${backend_address}/drawers/`)
    const data = await res.json()

    return data;
  }

  const fetchDrawer = async ( index) => {
    const res = await fetch(`${backend_address}/drawers/${index}`)
    const data = await res.json()

    return data;
  }

  const renameDrawer = (drawer) => {
    console.log(drawer)
    // const res = await fetch(`${backend_address}/drawers/${drawer.drawer_controller_id}`, {
    //   method: 'PUT',
    //   headers: {
    //     'Content-type': 'application/json'
    //   },
    //   body: JSON.stringify(drawer)
    // })
    // const data = await res.json()
    // console.log(data)
    // setDrawers(fetchDrawers())
  }
  

const sendGoal = async (goalID) => {
  console.log(goalID)
  const res = await fetch(`${backend_address}/robot/goal/?id=${goalID}`, {  
      method: 'PUT',
      headers: {
        'Content-type': 'application/json'
      },
      body: ""
  })
  const data = await res.json()
  console.log(data)
}
     

const changeStatus= async(statusID) =>  {
  console.log(statusID)
  const res = await fetch(`${backend_address}/robot/status/?status=${statusID}`, {
    method: 'PUT',
    headers: {
      'Content-type': 'application/json'
    },
    body: ""
  })
  const data = await res.json()
  console.log(data)
}

const openDrawer= async (drawer) => {
  console.log(drawer)
  const res = await fetch(`${backend_address}/drawer/${drawer.drawer_controller_id}/open/`, {
    method: 'PUT',
    headers: {
      'Content-type': 'application/json'
    },
    body: ""
  })
  const data = await res.json()
  console.log(data)
}

const toggleEmpty= async (drawer) => {
  console.log(drawer)
  const res = await fetch(`${backend_address}/drawer/${drawer.drawer_controller_id}/empty/?empty=${!drawer.empty}`, {
    method: 'PUT',
    headers: {
      'Content-type': 'application/json'
    },
  })
  const data = await res.json()
  console.log(data)
  updateDrawersFromDB()
}

const tabData=[
  { id:0, content:  <RobotControl mapPositions= {mapPositions} sendGoal={sendGoal} addMapPosition={addMapPosition} robotStatusChange= {changeStatus} /> , label: "Roboter Steuern"},
  { id:1, content:  <DrawerControl drawers= {drawers} renameDrawer= {renameDrawer} openDrawer={openDrawer} getDrawers= {fetchDrawers} toggleEmpty={toggleEmpty} /> , label: "Schubladen Öffnen"}
  ];

  return (

    <Router>
      
      <div className='container' id="Map">
        <Header />
          
       
       
        <Switch>

        <Route path='/' exact render={(props) => (
          <>
             {<ControlSwitch Tablist= {tabData}/>}
          </>
        )} />
        
          <Route path='/about' component={About}></Route>
        </Switch>
        <Footer />
      </div>
    </Router>
  );
}


export default App;


