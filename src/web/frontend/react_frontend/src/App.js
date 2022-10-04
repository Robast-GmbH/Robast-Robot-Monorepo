import Header from './components/Header.js'
import { BrowserRouter as Router, Route, Switch } from 'react-router-dom'
import { useState, useEffect,useRef } from 'react'
import Footer from './components/Footer.js'
import About from './components/About.js'
import React from 'react'
import ControlSwitch from './components/ControlSwitch.js'
import RobotControl from './components/RobotControl.js'
import DrawerControl from './components/DrawerControl.js'
import Stack from '@mui/material/Stack';
import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';

const backend_address = `10.10.23.6:8000`
function App() {

  const [mapPositions, setMapPositions] = useState([])
  const [drawers, setDrawers]= useState([])
  const [user, setUser] = useState({id: 0 , name: "Guest", admin: false})


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

  useEffect(() => {
    const getUser = async () => {
    const userFromServer = await fetchUser()
    if (userFromServer!="")
    {
      setUser(userFromServer)
    }
   }
    getUser()
  }, [])
  
  
  
  const fetchUser = async () => {
    
    const user_id= sessionStorage.getItem('token')
    if(user_id=="undefined")
    {
      return ""
    }
    const res = await fetch(`${backend_address}/users/${user_id}`)
    const data = await res.json()  
    if(res.ok)
      return data;
    else
      return ""
  }

 

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

  const renameDrawer = async (drawer) => {
    console.log(drawer)
    const res = await fetch(`${backend_address}/drawers/${drawer.drawer_controller_id}`, {
      method: 'PUT',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(drawer)
    })
    const data = await res.json()
    console.log(data)
    setDrawers(fetchDrawers())
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

const userLogIn=async (user)=>{
  console.log(user)
  const res = await fetch(`${backend_address}/users/login`,{
  method: 'POST',
  headers: {
    'Content-type': 'application/json'
  },
  body: JSON.stringify(user)
})

    const data = await res.json()
    console.log(data)
    if(data!= null){
      console.log(data)
      sessionStorage.setItem('token',data.id );
      setUser(data)
      return true
    }
    return false
  
}

const userLogOut= ()=>{
  console.log("logout")
  sessionStorage.removeItem('token');
  window.parent.location.reload(false)
}

const theme = useTheme();
const matches = useMediaQuery(theme.breakpoints.up('sm'));

const tabData=[
  { id:0, content:  <RobotControl  user={user} mapPositions= {mapPositions} sendGoal={sendGoal} addMapPosition={addMapPosition} robotStatusChange= {changeStatus} /> , label: (matches?"Roboter steuern": "Steuern") },
  { id:1, content:  <DrawerControl user={user} drawers= {drawers} renameDrawer= {renameDrawer} openDrawer={openDrawer} getDrawers= {fetchDrawers} toggleEmpty={toggleEmpty} /> , label: (matches? "Schubladen öffnen" : "Schubladen")}
  ];

  return (

    <Router>
      
      <Stack className= {matches? "container": "Mcontainer"} id="Map" direction="row">
        <Stack id="main" sx={{ flexGrow: 1 }}>
          <Header user= {user} login= {userLogIn} logout={ userLogOut} />
          <Switch>
            <Route path='/' exact render={(props) => (
              <>
                {<ControlSwitch Tablist= {tabData}/>}
              </>
              )} />
        
            <Route path='/about' component={About}></Route>
          </Switch>
        </Stack>
       {/* <Footer />*/}
      </Stack>
    </Router>
  );
}


export default App;


