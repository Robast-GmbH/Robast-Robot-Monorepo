import { useState, useEffect } from 'react'

const DebugRequester = () => {
    const [debug, setDebug] = useState([

    ])
    useEffect(() => {
        const getDebug = async () => {
          const debugFromServer = await fetchDebug()
          setDebug(debugFromServer)
        }
    
        getDebug()
      }, [])
      const fetchDebug = async () => {
        const res = await fetch(`http://localhost:8000/users/?skip=0&limit=100`)
        const data = await res.json()
    
        return data;
      }


    return (
        <div>
            <h1>
                {debug.map(debug => (
                    <li key={debug.email}>{debug.email}</li>
                ))}
            </h1>
        </div>
    )
}

export default DebugRequester
