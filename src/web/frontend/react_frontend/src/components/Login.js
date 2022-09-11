import { useState } from 'react'
import PropTypes from 'prop-types'


const Login = ({login}) => {
    const [name, setUserName] = useState('')
    const [hashed_password, setPassword] = useState('')

        
    const onSubmit =async (e) => {
                e.preventDefault()
                if (!name) {
                        alert('Bitte den Benutzernamen eintragen')
                        return
                }
                
                if (!hashed_password) {
                        alert('Bitte das Passwort eingeben') 
                        return
                }
              const sucessful = await login( { name , hashed_password })
                if(!sucessful)
                {
                        alert('Passwort und Benutzername sind nicht richtig. ') 
                        return          
                }
                console.log(sucessful)
                setUserName(0)
                setPassword("")
                //window.parent.location.reload(false)
        }
              
        return (
                <form className='rename-form' onSubmit={onSubmit}>
                        

                        <div className='form-control'>
                                <label> Benutzername</label>
                                <input type='text' value={name} onChange={(e) => setUserName(e.target.value)} />
                        </div>
                        <div className='form-control'>
                                <label> Passwort</label>
                                <input type='password' value={hashed_password} onChange={(e) => setPassword(e.target.value)} />
                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        
)}
export default Login