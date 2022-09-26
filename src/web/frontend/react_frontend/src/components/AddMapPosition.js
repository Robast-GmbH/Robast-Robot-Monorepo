import { useState } from 'react'
import PropTypes from 'prop-types'
import Stack from '@mui/material/Stack';
import TextField from '@mui/material/TextField';
import Button from '@mui/material/Button';

const AddMapPosition = ({ onAdd }) => {
        const [name, setPositionItemTitle] = useState('')
        const [x, setPositionXCoordinate] = useState(0.0)
        const [y, setPositionYCoordinate] = useState(0.0)
        const [t, setTheta] = useState(0.0)

        const [name_error, setTitleError] = useState("")
        const [x_error, setXError] = useState("")
        const [y_error, setYError] = useState("")
        const [t_error, setTError] = useState("")



        const onSubmit = (e) => {
                e.preventDefault()
                if (!name) {
                        alert('Add a Title')
                        return
                }
                if (!x) {
                        alert('Add a X-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                if (!y) {
                        alert('Add a Y-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                if (!t) {
                        alert('Add Theta') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                
                onAdd({ name, x, y, t})
                setPositionItemTitle('')
                setPositionXCoordinate(0.0)
                setPositionYCoordinate(0.0)
                setTheta(0.0)
                window.parent.location.reload(false)
        }

        return (
                <Stack spacing={2} className="popup">
                      <h2>Neues Ziel</h2> 
                     <TextField id="new_point_name" label="name" variant="outlined" onChange={(e) => setPositionItemTitle(e.target.value)}/>
                     <TextField id="new_point_x" label="x" variant="outlined" onChange={(e) => setPositionXCoordinate(parseFloat(e.target.value))} />
                     <TextField id="new_point_y" label="y" variant="outlined" onChange={(e) => setPositionYCoordinate(parseFloat(e.target.value))} />
                     <TextField id="new_point_t" label="t" variant="outlined" onChange={(e) => setTheta(parseFloat(e.target.value))} />
                     <Button variant="contained" onClick={onSubmit} >Speichern</Button>
                </Stack>
      
        )
}
AddMapPosition.propTypes = {
        onAdd: PropTypes.func
}
export default AddMapPosition


