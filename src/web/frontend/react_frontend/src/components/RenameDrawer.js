import { useState } from 'react'
import PropTypes from 'prop-types'
import Drawer from './Drawer'

import InputLabel from '@mui/material/InputLabel';
import Button from '@mui/material/Button';
import Stack from '@mui/material/Stack';
import TextField from '@mui/material/TextField';
import Select from '@mui/material/Select';
import MenuItem from '@mui/material/MenuItem';
import FormControl from '@mui/material/FormControl';



const RenameDrawer = ({ renameDrawer, drawers }) => {
        const [selected_drawer_id, setselectedDrawerID] = useState(0)
        const [new_name, setNewName] = useState('')

        const onSubmit = (e) => {
                e.preventDefault()
                if (!selected_drawer_id) {
                        alert('Wähle eine Schublade')
                        return
                }
                if (!new_name) {
                        alert('Wähle eien neuen Namen') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                const newDrawer=drawers.find(d=>d.id===selected_drawer_id)
                console.log(newDrawer)
                newDrawer.content=new_name
                renameDrawer(newDrawer)
                setselectedDrawerID(0)
                setNewName("")
                sessionStorage.setItem('tap-value', 1);
                window.parent.location.reload(false)
        }
        
        return (


                <Stack spacing={3} className="popup"> 
                <h2 id="titel_rename">Schublade umbenennen </h2>
               

                <FormControl fullWidth>
                        <InputLabel id="demo-simple-select-label">Schublade</InputLabel>
                        <Select
                labelId="demo-simple-select-label"
                id="demo-simple-select"
                value={selected_drawer_id}
                label="Schublade"
                onChange= {(event) => { setselectedDrawerID(event.target.value)}}
                >
                       
                        {drawers.map((drawer)=>{
                             return   <MenuItem value={ drawer.id}>{drawer.drawer_controller_id + " (" + drawer.content + " )"} </MenuItem>
                        })}

                </Select>
                </FormControl>

                { selected_drawer_id>0?   (<TextField  label ="Alte Füllung" defaultValue={drawers.find(d=>d.id===selected_drawer_id).content} disabled />):("") }  
                <TextField id="content_rename" label="Füllung" variant="outlined" onChange={(e) => setNewName(e.target.value)} />
  
                <Button variant="contained" onClick={onSubmit} >Speichern</Button>
           </Stack>
                
        )
}
RenameDrawer.propTypes = {
        renameDrawer: PropTypes.func,
        drawers: PropTypes.arrayOf(Drawer),
}
export default RenameDrawer

