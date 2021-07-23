import Header from './components/Header.js'
import { BrowserRouter as Router, Route } from 'react-router-dom'
import { useState, useEffect } from 'react'
import AddTask from './components/AddTask.js'
import Footer from './components/Footer.js'
import About from './components/About.js'
import Popup from 'reactjs-popup';
import RosMap from './components/RosMap.js'
import ShowTasks from './components/ShowTasks.js'


function App() {
  const [showAddTask, setShowAddTask] = useState(false)
  const [showOrder, setShowOrder] = useState(false)
  const [popupIsOpen, setPopupIsOpen] = useState(false)
  const [tasks, setTasks] = useState([

  ])
  const [OrderCoords, setOrderCoords] = useState([1,2])

  const closeModal = () => setPopupIsOpen(false);
  const openModal = () => setPopupIsOpen(true);

  useEffect(() => {
    const getTasks = async () => {
      const tasksFromServer = await fetchTasks()
      setTasks(tasksFromServer)
    }

    getTasks()
  }, [])


  const fetchTasks = async () => {
    const res = await fetch('http://localhost:5000/tasks')
    const data = await res.json()

    return data;
  }

  const fetchTask = async (id) => {
    const res = await fetch(`http://localhost:5000/tasks/${id}`)
    const data = await res.json()

    return data;
  }

  //DELETE
  const deleteTask = async (id) => {
    await fetch(`http://localhost:5000/tasks/${id}`, { method: 'DELETE' })

    setTasks(tasks.filter((task) => task.id !== id));
  }

  const toggleReminder = async (id) => {
    const taskToToggle = await fetchTask(id)
    const updatedTask = { ...taskToToggle, reminder: !taskToToggle.reminder }

    const res = await fetch(`http://localhost:5000/tasks/${id}`, {
      method: 'PUT',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(updatedTask)
    })

    const data = await res.json()
    setTasks(
      tasks.map((task) =>
        task.id === id ? { ...task, reminder: data.reminder } : task))
  }

  const addTask = async (task) => {
    const res = await fetch('http://localhost:5000/tasks', {
      method: 'POST',
      headers: {
        'Content-type': 'application/json'
      },
      body: JSON.stringify(task)
    })
    const data = await res.json()
    console.log(data)
    setTasks([...tasks, data])
    closeModal()
  }

  const showCoords = (event) => {

    openModal();
    const rect = document.getElementById('RosMap').getBoundingClientRect();
    const map_x = event.clientX - rect.left;
    const map_y = event.clientY - rect.top;
    setOrderCoords([map_x, map_y]);
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
            {showOrder  && <ShowTasks tasks={tasks} onDelete={deleteTask} onToggle={toggleReminder}></ShowTasks>}
            {<h1>click on the location to order too</h1>}

            <RosMap onClick={showCoords} />
            <Popup open={popupIsOpen} closeOnDocumentClick onClose={closeModal}>
              <>
                {<AddTask onAdd={addTask} coords={OrderCoords} />}
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
