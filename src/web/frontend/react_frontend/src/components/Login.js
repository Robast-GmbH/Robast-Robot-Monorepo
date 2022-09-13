import { useState } from 'react'
import Stack from '@mui/material/Stack';
import TextField from '@mui/material/TextField';
import Button from '@mui/material/Button';


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
                <Stack spacing={4}>
                        <h2> Login </h2>
                        
                        <TextField id="outlined-basic" label="Benutzername" variant="outlined" onChange={(e) => setUserName(e.target.value)} />
                        <TextField id="outlined-basic1" type='password' label="Passwort" variant="outlined" onChange={(e) => setPassword(e.target.value)} />
                        <Button variant="contained" onClick={onSubmit} >login</Button>

                </Stack>
                               
                
)}
export default Login
